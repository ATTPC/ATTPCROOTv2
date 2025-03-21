#include "At3DBraggCurveHandler.h"

#include "AtFindVertex.h"
#include "AtHit.h"
#include "AtPad.h"
#include "AtPatternLine.h"
#include "AtRawEvent.h"
#include "AtTrack.h"

#include <FairLogger.h>

#include <Math/Point3D.h>
#include <Rtypes.h>
#include <TDirectory.h>
#include <TMath.h>

#include <cmath>

using XYZPoint = ROOT::Math::XYZPoint;

At3DBraggCurveHandler::At3DBraggCurveHandler()
{
   fPSA = new AtPSAHitPerTBInRegion();
   fPSA->Init();
   fPSA->SetThreshold(0);
}

At3DBraggCurveHandler::~At3DBraggCurveHandler() = default;

XYZPoint At3DBraggCurveHandler::FindVertex(AtTrack &track)
{
   std::vector<AtTrack> trackToFindVtx;
   trackToFindVtx.push_back(track);
   AtFindVertex findVtx(30);
   findVtx.FindVertex(trackToFindVtx, 1);
   std::vector<tracksFromVertex> tv = findVtx.GetTracksVertex();
   if (tv.size() != 1) {
      LOG(info) << "Found " << tv.size() << " vertex. We need to have 1 and only 1!";
      XYZPoint ret(-999, -999, -999);
      return ret;
   }
   return (XYZPoint)tv.at(0).vertex;
}

void At3DBraggCurveHandler::SetTrack(AtTrack &track)
{
   XYZPoint vertex = FindVertex(track);
   if (vertex.X() == -999 && vertex.Y() == -999 && vertex.Z() == -999)
      return;

   auto *pattern = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
   std::vector<AtHit> hitArray = track.GetHitArrayObject();
   for (auto hit : hitArray) {
      XYZPoint closestPatternPoint = pattern->ClosestPointOnPattern(hit.GetPosition());

      Double_t archlength = TMath::Sqrt(std::pow(closestPatternPoint.X() - vertex.X(), 2) +
                                        std::pow(closestPatternPoint.Y() - vertex.Y(), 2) +
                                        std::pow(closestPatternPoint.Z() - vertex.Z(), 2));

      track.Add3DBraggCurvePair(std::pair<Double_t, Double_t>(archlength, hit.GetTraceIntegral()));
   }
}

void At3DBraggCurveHandler::SetTrack(AtTrack &track, AtRawEvent *rawEvent)
{
   XYZPoint vertex = FindVertex(track);
   if (vertex.X() == -999 && vertex.Y() == -999 && vertex.Z() == -999)
      return;

   auto *pattern = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
   std::vector<AtHit> hitArray = track.GetHitArrayObject();
   for (auto hit : hitArray) {
      Int_t iPad = hit.GetPadNum();
      Int_t centralTS = hit.GetTimeStamp();

      AtPad *pad = rawEvent->GetPad(iPad);
      if (pad == nullptr) {
         LOG(error) << "Somehow, an AtHit has an pad number that does not correspond with any of the AtRawEvent. "
                       "Skipping this hit!";
         continue;
      }

      Int_t minTS = centralTS - fTSSemiWidth;
      Int_t maxTS = centralTS + fTSSemiWidth;

      fPSA->SetTBLimits(std::pair<Int_t, Int_t>(minTS, maxTS));

      auto subHitVector = fPSA->AnalyzePad(pad);
      for (auto &&subHit : subHitVector) {
         XYZPoint closestPatternPoint = pattern->ClosestPointOnPattern(subHit->GetPosition());

         Double_t archlength = TMath::Sqrt(std::pow(closestPatternPoint.X() - vertex.X(), 2) +
                                           std::pow(closestPatternPoint.Y() - vertex.Y(), 2) +
                                           std::pow(closestPatternPoint.Z() - vertex.Z(), 2));

         track.Add3DBraggCurvePair(std::pair<Double_t, Double_t>(archlength, subHit->GetTraceIntegral()));
      }
   }
}
