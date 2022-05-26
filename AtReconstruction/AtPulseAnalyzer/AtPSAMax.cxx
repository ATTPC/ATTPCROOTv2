#include "AtPSAMax.h"

#include <FairLogger.h>

// AtTPCROOT classes
#include "AtCalibration.h"
#include "AtEvent.h"
#include "AtHit.h"
#include "AtPad.h"
#include "AtRawEvent.h"

// ROOT classes
#include <Math/Point2D.h>

// STL
#include <algorithm>
#include <array> // for array
#include <cmath>
#include <iostream> // for basic_ostream::operator<<
#include <map>
#include <memory> // for unique_ptr, make_unique
#include <numeric>
#include <utility> // for pair
#include <vector>  // for vector
//#ifdef _OPENMP
//#include <omp.h>
//#endif
using XYZPoint = ROOT::Math::XYZPoint;

ClassImp(AtPSAMax);
Double_t AtPSAMax::getThreshold(int padSize)
{
   if (padSize == 0)
      return fThresholdlow; // threshold for central pads
   else
      return fThreshold; // threshold for big pads (or all other not small)
}

void AtPSAMax::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{
   Double_t QEventTot = 0.0;
   Double_t RhoVariance = 0.0;
   Double_t RhoMean = 0.0;
   Double_t Rho2 = 0.0;

   std::map<Int_t, Int_t> PadMultiplicity;
   std::array<Float_t, 512> mesh{};
   mesh.fill(0);

   for (const auto &pad : rawEvent->GetPads()) {

      LOG(debug) << "Running PSA on pad " << pad->GetPadNum();
      Int_t PadNum = pad->GetPadNum();

      XYZPoint pos(pad->GetPadCoord().X(), pad->GetPadCoord().Y(), 0);

      if (pos.X() < -9000 || pos.Y() < -9000) {
         LOG(debug) << "Skipping pad, position is invalid";
         continue;
      }

      if (!(pad->IsPedestalSubtracted())) {
         LOG(ERROR) << "Pedestal should be subtracted to use this class!";
      }

      std::array<Double_t, 512> floatADC = pad->GetADC();
      auto maxAdcIt = std::max_element(floatADC.begin() + 20, floatADC.end() - 12);
      Int_t maxAdcIdx = std::distance(floatADC.begin(), maxAdcIt);

      if (shouldSaveHit(*maxAdcIt, getThreshold(pad->GetSizeID()), maxAdcIdx)) {

         // Calculation of the mean value of the peak time by interpolating the pulse
         Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                            (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);
         Double_t TBCorr = getTBCorr(floatADC, maxAdcIdx);
         Double_t QHitTot = std::accumulate(floatADC.begin(), floatADC.end(), 0);

         if (fIsTimeCorr)
            pos.SetZ(CalculateZGeo(TBCorr));
         else
            pos.SetZ(CalculateZGeo(maxAdcIdx));

         auto &hit = event->AddHit(PadNum, pos, *maxAdcIt);
         LOG(debug) << "Added hit with ID" << hit.GetHitID();

         hit.SetTimeStamp(maxAdcIdx);
         hit.SetTimeStampCorr(TBCorr);
         hit.SetTimeStampCorrInter(timemax);
         hit.SetTraceIntegral(QHitTot);

         Rho2 += pos.Mag2();
         RhoMean += pos.Rho();
         QEventTot += QHitTot;

         // Tracking MC points
         auto mcPointsMap = rawEvent->GetSimMCPointMap();
         if (mcPointsMap.size() > 0) {
            LOG(debug) << "MC Simulated points Map size " << mcPointsMap.size();
            TrackMCPoints(mcPointsMap, hit);
         }

         for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
            mesh[iTb] += floatADC[iTb];

      } // Valid Threshold

      PadMultiplicity.insert(std::pair<Int_t, Int_t>(pad->GetPadNum(), 1));

   } // Pad Loop

   // RhoVariance = Rho2 - (pow(RhoMean, 2) / (event->GetNumHits()));
   RhoVariance = Rho2 - (event->GetNumHits() * pow((RhoMean / event->GetNumHits()), 2));

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      event->SetMeshSignal(iTb, mesh[iTb]);
   event->SortHitArrayTime();
   event->SetMultiplicityMap(PadMultiplicity);
   event->SetRhoVariance(RhoVariance);
   event->SetEventCharge(QEventTot);
}
bool AtPSAMax::shouldSaveHit(double charge, double threshold, int tb)
{
   bool ret = true;
   if (threshold > 0 && charge < threshold) {
      ret = false;
      LOG(debug) << "Invalid threshold with charge: " << charge << " and threshold: " << threshold;
   }
   if ((tb < 20 || tb > 500)) {
      ret = false;
      LOG(debug) << "Peak is outside valid time window (20,500) TBs.";
   }

   return ret;
}

Double_t AtPSAMax::getTBCorr(AtPad::trace &adc, int maxAdcIdx)
{
   if (maxAdcIdx < 11)
      return 0;

   Double_t qTot = 0;
   Double_t tbAvg = 0;
   for (Int_t i = 0; i < 11; i++)
      if (adc[maxAdcIdx - i + 10] > 0 && adc[maxAdcIdx - i + 10] < 4000) {
         auto tb = maxAdcIdx - i + 5;
         qTot += adc[tb];
         tbAvg += adc[tb] / qTot * (tb - tbAvg);
      }
   return tbAvg;
}
