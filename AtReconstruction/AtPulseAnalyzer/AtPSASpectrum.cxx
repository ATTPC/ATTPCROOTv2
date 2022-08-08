#include "AtPSASpectrum.h"

#include "AtHit.h"
#include "AtPad.h"

#include <FairLogger.h>

#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <TSpectrum.h>

#include <array> // for array
#include <cmath>
#include <memory> // for unique_ptr, make_unique
#include <numeric>
#include <utility> // for pair

//#ifdef _OPENMP
//#include <omp.h>
//#endif
using XYZPoint = ROOT::Math::XYZPoint;
ClassImp(AtPSASpectrum);
AtPSASpectrum::HitVector AtPSASpectrum::AnalyzePad(AtPad *pad)
{
   LOG(debug) << "Running PSA on pad " << pad->GetPadNum();
   Int_t PadNum = pad->GetPadNum();
   double padThreshold = getThreshold(pad->GetSizeID());

   XYZPoint pos{pad->GetPadCoord().X(), pad->GetPadCoord().Y(), 0};
   XYZPoint posCorr{0, 0, 0};

   if (pos.X() < -9000 || pos.Y() < -9000) {
      LOG(debug) << "Skipping pad, position is invalid";
      return {};
   }

   if (!(pad->IsPedestalSubtracted())) {
      LOG(ERROR) << "Pedestal should be subtracted to use this class!";
   }

   auto adc = pad->GetADC();
   std::array<double, 512> floatADC = adc;
   std::array<double, 512> dummy{};
   floatADC.fill(0);
   dummy.fill(0);

   double traceIntegral = std::accumulate(adc.begin(), adc.end(), 0.0);

   auto PeakFinder = std::make_unique<TSpectrum>();
   auto numPeaks =
      PeakFinder->SearchHighRes(floatADC.data(), dummy.data(), fNumTbs, 4.7, 5, fBackGroundSuppression, 3, kTRUE, 3);

   if (fBackGroundInterp) {
      subtractBackground(floatADC);
   }
   HitVector hits;
   // Create a hit for each peak
   for (Int_t iPeak = 0; iPeak < numPeaks; iPeak++) {

      auto maxAdcIdx = (Int_t)(ceil((PeakFinder->GetPositionX())[iPeak]));
      if (maxAdcIdx < 3 || maxAdcIdx > 509)
         continue; // excluding the first and last 3 tb

      auto charge = floatADC[maxAdcIdx];
      if (padThreshold > 0 && charge < padThreshold) {
         LOG(debug) << "Invalid threshold with charge: " << charge << " and threshold: " << padThreshold;
         continue;
      }

      // Calculation of the mean value of the peak time by interpolating the pulse
      Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                         (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);

      Double_t TBCorr = calcTbCorrection(floatADC, maxAdcIdx);

      if (fIsTimeCorr)
         pos.SetZ(CalculateZGeo(TBCorr));
      else
         pos.SetZ(CalculateZGeo(maxAdcIdx));

      auto hit = std::make_unique<AtHit>(PadNum, pos, charge);

      hit->SetTimeStamp(maxAdcIdx);
      hit->SetTimeStampCorr(TBCorr);
      hit->SetTimeStampCorrInter(timemax);
      hit->SetTraceIntegral(traceIntegral);
      // TODO: The charge of each hit is the total charge of the spectrum, so for double
      // structures this is unrealistic.

      hits.push_back(std::move(hit));
   } // End loop over peaks
   return hits;
}

/**
 * Perform a background subtraction using TSpectrum
 */
void AtPSASpectrum::subtractBackground(std::array<Double_t, 512> &adc)
{
   auto bg = adc;
   auto BGInter = std::make_unique<TSpectrum>();
   BGInter->Background(bg.data(), fNumTbs, 6, TSpectrum::kBackDecreasingWindow, TSpectrum::kBackOrder2, kTRUE,
                       TSpectrum::kBackSmoothing7, kTRUE);

   for (Int_t iTb = 1; iTb < fNumTbs; iTb++) {
      adc[iTb] = adc[iTb] - bg[iTb];
      if (adc[iTb] < 0)
         adc[iTb] = 0;
   }
}
double AtPSASpectrum::calcTbCorrection(const std::array<Double_t, 512> &floatADC, int maxAdcIdx)
{
   double TBCorr = 0;
   double qTot = 0;
   if (maxAdcIdx > 11) {
      for (Int_t i = 0; i < 11; i++) {

         if (floatADC[maxAdcIdx - i + 10] > 0 && floatADC[maxAdcIdx - i + 10] < 4000) {
            TBCorr += (floatADC[maxAdcIdx - i + 5]) * (maxAdcIdx - i + 5);
            qTot += floatADC[maxAdcIdx - i + 5];
         }
      }
   }
   return TBCorr / qTot;
}
