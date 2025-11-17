#include "AtPSASi.h"

#include "AtHit.h"
#include "AtPad.h"

#include <FairLogger.h>

#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint

#include <algorithm>
#include <array>    // for array
#include <iterator> // for distance
#include <memory>   // for unique_ptr, make_unique
#include <numeric>
#include <utility> // for pair

// #ifdef _OPENMP
// #include <omp.h>
// #endif
using XYZPoint = ROOT::Math::XYZPoint;

AtPSASi::HitVector AtPSASi::AnalyzePad(AtPad *pad)
{
   XYZPoint pos(0, 0, 0);

   if (!(pad->IsPedestalSubtracted())) {
      LOG(error) << "Pedestal should be subtracted to use this class!";
   }

   std::array<Double_t, 512> floatADC = pad->GetADC();
   auto maxAdcIt = std::max_element(floatADC.begin() + 20, floatADC.end() - 12);
   Int_t maxAdcIdx = std::distance(floatADC.begin(), maxAdcIt);

   if (!shouldSaveHit(*maxAdcIt, fThreshold, maxAdcIdx))
      return {};

   // Calculation of the mean value of the peak time by interpolating the pulse
   Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                      (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);
   Double_t TBCorr = getTBCorr(floatADC, maxAdcIdx);
   Double_t QHitTot = std::accumulate(floatADC.begin(), floatADC.end(), 0);

   auto hit = std::make_unique<AtHit>(pad->GetPadNum(), pos, *maxAdcIt);

   hit->SetTimeStamp(maxAdcIdx);
   hit->SetTimeStampCorr(TBCorr);
   hit->SetTimeStampCorrInter(timemax);
   hit->SetTraceIntegral(QHitTot);

   HitVector ret;
   ret.push_back(std::move(hit));
   return ret;
}
bool AtPSASi::shouldSaveHit(double charge, double threshold, int tb)
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

Double_t AtPSASi::getTBCorr(AtPad::trace &adc, int maxAdcIdx)
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

AtPSASi::HitVector AtPSASi::AnalyzeGenTrace(AtGenericTrace *genTrace)
{
   XYZPoint pos(0, 0, 0);

   std::array<Double_t, 512> floatADC;
   std::vector<Double_t> floatADCVector = genTrace->GetADC();
   for(int i=0; i<512; i++) 
	   floatADC[i] = floatADCVector[i];
   auto maxAdcIt = std::max_element(floatADC.begin() + 20, floatADC.end() - 12);
   Int_t maxAdcIdx = std::distance(floatADC.begin(), maxAdcIt);

   if (!shouldSaveHit(*maxAdcIt, fThreshold, maxAdcIdx))
      return {};

   // Calculation of the mean value of the peak time by interpolating the pulse
   Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                      (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);
   Double_t TBCorr = getTBCorr(floatADC, maxAdcIdx);
   Double_t QHitTot = std::accumulate(floatADC.begin(), floatADC.end(), 0);

   auto hit = std::make_unique<AtHit>(0, pos, *maxAdcIt);

   hit->SetTimeStamp(maxAdcIdx);
   hit->SetTimeStampCorr(TBCorr);
   hit->SetTimeStampCorrInter(timemax);
   hit->SetTraceIntegral(QHitTot);

   HitVector ret;
   ret.push_back(std::move(hit));
   return ret;
}

ClassImp(AtPSASi);
