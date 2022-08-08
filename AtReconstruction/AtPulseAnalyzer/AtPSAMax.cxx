#include "AtPSAMax.h"

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

//#ifdef _OPENMP
//#include <omp.h>
//#endif
using XYZPoint = ROOT::Math::XYZPoint;

AtPSAMax::HitVector AtPSAMax::AnalyzePad(AtPad *pad)
{
   XYZPoint pos(pad->GetPadCoord().X(), pad->GetPadCoord().Y(), 0);

   if (pos.X() < -9000 || pos.Y() < -9000) {
      LOG(debug) << "Skipping pad, position is invalid";
      return {};
   }

   if (!(pad->IsPedestalSubtracted())) {
      LOG(ERROR) << "Pedestal should be subtracted to use this class!";
   }

   std::array<Double_t, 512> floatADC = pad->GetADC();
   auto maxAdcIt = std::max_element(floatADC.begin() + 20, floatADC.end() - 12);
   Int_t maxAdcIdx = std::distance(floatADC.begin(), maxAdcIt);

   if (!shouldSaveHit(*maxAdcIt, getThreshold(pad->GetSizeID()), maxAdcIdx))
      return {};

   // Calculation of the mean value of the peak time by interpolating the pulse
   Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                      (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);
   Double_t TBCorr = getTBCorr(floatADC, maxAdcIdx);
   Double_t QHitTot = std::accumulate(floatADC.begin(), floatADC.end(), 0);

   if (fIsTimeCorr)
      pos.SetZ(CalculateZGeo(TBCorr));
   else
      pos.SetZ(CalculateZGeo(maxAdcIdx));

   auto hit = std::make_unique<AtHit>(pad->GetPadNum(), pos, *maxAdcIt);

   hit->SetTimeStamp(maxAdcIdx);
   hit->SetTimeStampCorr(TBCorr);
   hit->SetTimeStampCorrInter(timemax);
   hit->SetTraceIntegral(QHitTot);

   HitVector ret;
   ret.push_back(std::move(hit));
   return ret;
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

ClassImp(AtPSAMax);
