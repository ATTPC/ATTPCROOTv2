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

   std::array<Double_t, 256> floatADC;
   std::vector<Double_t> floatADCVector = genTrace->GetADC();
   // std::cout << "GAGG ADC entries: " << floatADCVector.size() <<  std::endl;
   if (floatADCVector.size() >= 256) {
      for (int i = 0; i < 256; i++)
         floatADC[i] = floatADCVector[i];
   } else {
      LOG(error) << "There are not 256 ADC values in the GAGG trace. Skipping!";
      return {};
   }

   // Get baseline value.
   double baseline{};
   for (int i = 10; i < 20; i++)
      baseline += floatADC[i];
   baseline /= 10;
   // std::cout << "baseline = " << baseline <<  std::endl;

   Double_t *maxAdcIt{nullptr};
   Double_t charge{-999};
   if (fPositivePolarity) {
      maxAdcIt = std::max_element(floatADC.begin() + 20, floatADC.end() - 12);
      charge = *maxAdcIt - baseline;
   } else {
      maxAdcIt = std::min_element(floatADC.begin() + 20, floatADC.end() - 12);
      charge = -(*maxAdcIt - baseline);
   }
   Int_t maxAdcIdx = std::distance(floatADC.begin(), maxAdcIt);
   // std::cout << " Max ADC = " << *maxAdcIt << std::endl;

   // std::cout << " Diff ADC = " << *maxAdcIt - baseline << std::endl;

   if (!shouldSaveHit(charge, fThreshold, maxAdcIdx)) {
      LOG(debug) << "GAGG trace did not pass threshold.";
      return {};
   }

   // Calculation of the mean value of the peak time by interpolating the pulse
   Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                      (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);
   Double_t QHitTot = std::abs(std::accumulate(floatADC.begin(), floatADC.end(), 0)) - baseline * floatADC.size();

   auto hit = std::make_unique<AtHit>(0, pos, charge);

   hit->SetTimeStamp(maxAdcIdx);
   hit->SetTimeStampCorrInter(timemax);
   hit->SetTraceIntegral(QHitTot);

   HitVector ret;
   ret.push_back(std::move(hit));
   return ret;
}

ClassImp(AtPSASi);
