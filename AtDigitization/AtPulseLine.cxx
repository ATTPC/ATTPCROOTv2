#include "AtPulseLine.h"

#include "AtMCPoint.h"
#include "AtMap.h"
#include "AtSimulatedLine.h"
#include "AtSimulatedPoint.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <TAxis.h>
#include <TClonesArray.h>
#include <TH1.h>
#include <TH2Poly.h>
#include <TMath.h>
#include <TObject.h>
#include <TRandom.h>

#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>
constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
using XYPoint = ROOT::Math::XYPoint;

AtPulseLine::AtPulseLine(AtMapPtr map, ResponseFunc response) : AtPulse(map, response)
{
   LOG(debug) << "Constructor of AtPulseLineTask";
}

Int_t AtPulseLine::throwRandomAndGetPadAfterDiffusion(const ROOT::Math::XYZVector &loc, Double_t diffusionSigma)
{
   auto r = gRandom->Gaus(0, diffusionSigma);
   auto phi = gRandom->Uniform(0, TMath::TwoPi());
   Double_t propX = loc.x() + r * TMath::Cos(phi);
   Double_t propY = loc.y() + r * TMath::Sin(phi);
   XYPoint pos(propX, propY);
   return fMap->GetPadNum(pos);
}

void AtPulseLine::generateIntegrationMap(AtSimulatedLine &line)
{
   // MC the integration over the pad plane
   fXYintegrationMap.clear();
   auto loc = line.GetPosition();
   Int_t validPoints = 0;

   LOG(debug2) << "Sampling with transverse diffusion of: " << line.GetTransverseDiffusion();
   for (int i = 0; i < fNumIntegrationPoints; ++i) {
      auto padNumber = throwRandomAndGetPadAfterDiffusion(loc, line.GetTransverseDiffusion());

      if (padNumber < 0)
         continue;

      fXYintegrationMap[padNumber]++;
      validPoints++;
   }

   for (auto &elem : fXYintegrationMap)
      elem.second /= (double)validPoints;
}

bool AtPulseLine::AssignElectronsToPad(AtSimulatedPoint *point)
{
   auto line = dynamic_cast<AtSimulatedLine *>(point);
   if (line == nullptr) {
      LOG(fatal) << "Data in branch AtSimulatedPoint is not of type AtSimulatedLine!";
      return false;
   }

   generateIntegrationMap(*line);
   std::vector<double> zIntegration; // zero is binMin
   auto binMin = integrateTimebuckets(zIntegration, line);

   // Now loop through all pads in the integration map, and add electrons
   for (const auto &[padNum, percentEle] : fXYintegrationMap) {
      if (fMap->IsInhibited(padNum) == AtMap::InhibitType::kTotal)
         continue;

      for (int i = 0; i < zIntegration.size(); ++i) {
         auto zLoc = fPadCharge[0]->GetXaxis()->GetBinCenter(i + binMin);
         auto charge = line->GetCharge() * zIntegration[i] * percentEle;
         double gain = GetGain(padNum, charge);
         fPadCharge[padNum]->Fill(zLoc, gain * charge);
         fPadsWithCharge.insert(padNum);
      }
   }

   return true;
}
// Returns the bin ID (binMin) that the zIntegral starts from
// fills zIntegral with the integral for bins starting with binMin, inclusive
Int_t AtPulseLine::integrateTimebuckets(std::vector<double> &zIntegral, AtSimulatedLine *line)
{
   zIntegral.clear();

   // Shift the times by the pad plane location
   auto tMax = line->GetInitialPosition().z() + fTBPadPlane * fTBTime;
   auto tMin = line->GetFinalPosition().z() + fTBPadPlane * fTBTime;
   auto dT = (tMax - tMin);
   if (dT > fTBTime) {
      LOG(warn) << "This line charge spans multiple TB widths from " << tMin << " us to " << tMax << " us.";
      LOG(warn) << " The time distribution is likely widder than will be calculated. dT: " << dT
                << "ns TB width: " << fTBTime << "ns";
   }

   auto tMean = (tMax + tMin) / 2.;
   auto tIntegrationMinimum = tMin - fNumSigmaToIntegrateZ * line->GetLongitudinalDiffusion();
   auto tIntegrationMaximum = tMax + fNumSigmaToIntegrateZ * line->GetLongitudinalDiffusion();

   const TAxis *axis = fPadCharge[0]->GetXaxis();
   auto binMin = axis->FindBin(tIntegrationMinimum);
   auto binMax = axis->FindBin(tIntegrationMaximum);
   if (binMin < fTBPadPlane)
      binMin = fTBPadPlane;
   if (binMax > fTBEntrance)
      binMax = fTBEntrance;

   // Integrate G(tMean, sigmaLongDiff) over each time bucket from binMin to binMax
   // and fill zIntegral with the result.
   auto denominator = line->GetLongitudinalDiffusion() * TMath::Sqrt(2);
   double lowerBound = TMath::Erf((axis->GetBinLowEdge(binMin) - tMean) / denominator);
   for (int i = binMin; i <= binMax; ++i) {
      auto upperBound = TMath::Erf((axis->GetBinUpEdge(i) - tMean) / denominator);
      auto integral = 0.5 * (upperBound - lowerBound);
      zIntegral.emplace_back(integral);
      lowerBound = upperBound;
   }

   // Renormalize the integral if we're up against the pad plane or window
   if (binMin == fTBPadPlane || binMax == fTBEntrance) {
      auto sum = std::accumulate(zIntegral.begin(), zIntegral.end(), 0.);
      for (auto &elem : zIntegral)
         elem /= sum;
   }

   return binMin;
}
