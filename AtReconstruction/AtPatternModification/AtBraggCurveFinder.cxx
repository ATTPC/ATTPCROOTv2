#include "AtBraggCurveFinder.h"

#include "AtFindVertex.h"
#include "AtPattern.h"

#include <TH1F.h>

void AtBraggCurveFinder::InitializePSA()
{
   fPSA = std::make_unique<AtPSAHitPerTB>();
   fPSA->SetReplaceTraceIntegral(false);
   fPSA->Init();
}

void AtBraggCurveFinder::Init()
{
   if (fNeedPSA)
      InitializePSA();

   uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
   std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
   fRNG.seed(ss);
}

void AtBraggCurveFinder::ModifyPatternEvent(AtPatternEvent *patternEvent, AtRawEvent *rawEvent, AtEvent *event)
{
   if (rawEvent == nullptr)
      LOG(warning) << "No AtRawEvent was passed to the At3DBraggFinder, so it will not be used. The resulting bragg "
                      "curve may have more fluctuations in the ELoss per bin than expected!";

   AtPatternModification::ModifyPatternEvent(patternEvent, rawEvent, event);
}

AtTrack AtBraggCurveFinder::GetModifiedTrack(const AtTrack &track, AtRawEvent *rawEvent, AtEvent *event)
{
   // Create a copy of the AtTrack as an AtTrackBragg (not yet, still AtTrack for now).
   AtTrack modifiedTrack(track);

   // Extract the AtPattern.
   auto *pattern = modifiedTrack.GetPattern();

   // Find the vertex of this track.
   std::vector<AtTrack> trackToFindVtx;
   trackToFindVtx.push_back(modifiedTrack);
   AtFindVertex findVtx(fLineDistThreshold);
   findVtx.FindVertex(trackToFindVtx, 1);
   std::vector<tracksFromVertex> tv = findVtx.GetTracksVertex();
   if (tv.size() != 1) {
      LOG(warning) << "Found " << tv.size()
                   << " vertex. We need to have 1 and only 1 to find the Bragg curve! Skipping this track!";
      return modifiedTrack;
   }
   XYZPoint vertex = (XYZPoint)tv.at(0).vertex;

   // Extract the AtHits.
   std::vector<AtHit> hitArray = modifiedTrack.GetHitArrayObject();
   for (auto hit : hitArray) {
      ProcessHit(vertex, hit, modifiedTrack, rawEvent);
   }

   // Make the Bragg curve histogram.
   GenerateBraggCurveHistogram(modifiedTrack);

   // Return the modified track.
   return modifiedTrack;
}

void AtBraggCurveFinder::ProcessHit(XYZPoint vertex, AtHit hit, AtTrack &modifiedTrack, AtRawEvent *rawEvent)
{
   if (rawEvent == nullptr) {
      ProcessHit(vertex, hit, modifiedTrack);
      return;
   }

   if (fPSA == nullptr) {
      LOG(info) << " AtPSAHitPerTB was not initialized yet but it is needed. Initializing now...";
      InitializePSA();
   }

   Int_t iPad = hit.GetPadNum();
   Int_t centralTS = hit.GetTimeStamp();

   AtPad *pad = rawEvent->GetPad(iPad);
   if (pad == nullptr) {
      LOG(error) << "Somehow, an AtHit has an pad number that does not correspond with any of the AtRawEvent. Skipping "
                    "this hit!";
      return;
   }

   Int_t minTS = centralTS - fTSSemiWidth;
   Int_t maxTS = centralTS + fTSSemiWidth;
   fPSA->SetTBLimits(std::pair<Int_t, Int_t>(minTS, maxTS));

   auto subHitVector = fPSA->AnalyzePad(pad);
   for (auto &&subHit : subHitVector)
      ProcessHit(vertex, *subHit, modifiedTrack);
}

void AtBraggCurveFinder::ProcessHit(XYZPoint vertex, AtHit hit, AtTrack &modifiedTrack)
{
   // Extract yet again the AtPattern.
   auto *pattern = modifiedTrack.GetPattern();

   Double_t range = pattern->DistanceAlongPattern(vertex, hit.GetPosition());
   Double_t eLoss = hit.GetTraceIntegral();

   // TO-DO: Correct eLoss of big pads due to the difference in capacitance.

   modifiedTrack.AddBraggCurvePair(range, eLoss);
}

void AtBraggCurveFinder::GenerateBraggCurveHistogram(AtTrack &modifiedTrack)
{

   // Define the histogram where to integrate the charge over bins.
   int nBins = std::ceil(fMaxLength / fBinSize);
   TH1F *histBraggCurve = new TH1F("histBraggCurve", "histBraggCurve", nBins, 0, nBins * fBinSize);
   histBraggCurve->SetDirectory(0);

   // Start with a vector of 0s in order to sun new entries for smoothing.
   std::vector<Double_t> IntegratedELossValues(nBins, 0);
   for (int i = 0; i < fNumSmoothingSteps; i++) {

      // Fill in the histogram.
      auto braggCurveValues = modifiedTrack.GetBraggCurveValues();
      for (auto pairBragg : braggCurveValues) {
         double rangeRandomOffset = fUniform(fRNG) * fBinSize;
         histBraggCurve->Fill(pairBragg.first + rangeRandomOffset, pairBragg.second);
      }

      // Save this ELoss values in the vector.
      for (int j = 1; j <= histBraggCurve->GetNbinsX(); j++)
         IntegratedELossValues[j - 1] += histBraggCurve->GetBinContent(j) / fNumSmoothingSteps;

      // Reset histogram.
      histBraggCurve->Reset();
   }

   // Calculate the ELoss errors and get the range values.
   std::vector<Double_t> RangeValues;
   std::vector<Double_t> ELossErrors;
   for (int i = 0; i < nBins; i++) {
      RangeValues.push_back(histBraggCurve->GetBinCenter(i + 1));
      ELossErrors.push_back(IntegratedELossValues[i] * fELossRelativeError);
   }

   // Delete the TH1F to prevent memory leaks.
   delete histBraggCurve;

   // Create an AtTrack::BraggCurve struct where to save the Bragg curve information.
   AtTrack::BraggCurve braggCurve;
   braggCurve.IntegratedELossValues = IntegratedELossValues;
   braggCurve.RangeValues = RangeValues;
   braggCurve.ELossErrors = ELossErrors;
   braggCurve.nBins = nBins;
   braggCurve.binSize = fBinSize;
   braggCurve.smoothingSteps = fNumSmoothingSteps;

   modifiedTrack.SetBraggCurve(braggCurve);
}
