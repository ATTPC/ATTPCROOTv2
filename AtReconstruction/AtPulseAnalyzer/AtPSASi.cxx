#include "AtPSASi.h"

#include "AtHit.h"
#include "AtPad.h"

#include <FairLogger.h>

#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <TF1.h>
#include <TGraph.h>
#include <TMath.h>

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

AtPSASi::AtPSASi() : AtPSA()
{
   fLowBLRegion[0] = -50;
   fLowBLRegion[1] = -30;
   fHighBLRegion[0] = 80;
   fHighBLRegion[1] = 100;
   fEnergyIntegral[0] = -10;
   fEnergyIntegral[1] = 15;
   fFitClipped = false;
   fOverflowReconstruction = true;
   fDumpTraces = false;
   fMinIndx = 5;
}

void AtPSASi::SetLowBLRegion(int low, int hi)
{
   fLowBLRegion[0] = low;
   fLowBLRegion[1] = hi;
}

void AtPSASi::SetHighBLRegion(int low, int hi)
{
   fHighBLRegion[0] = low;
   fHighBLRegion[1] = hi;
}

void AtPSASi::SetEnergyIntegral(int low, int hi)
{
   fEnergyIntegral[0] = low;
   fEnergyIntegral[1] = hi;
}

void AtPSASi::SetFitClipped(bool fit)
{
   fFitClipped = fit;
}

void AtPSASi::SetDumpTraces(bool dump)
{
   if (dump) {
      std::cout << "Warning! SetDumpTraces will make potentially very large ASCII files! This should only be used for "
                   "debugging purposes!"
                << std::endl;
   }
   fDumpTraces = dump;
}

void AtPSASi::SetOverflowReconstruction(bool overflow)
{
   fOverflowReconstruction = overflow;
}

AtPSASi::HitVector AtPSASi::AnalyzePad(AtPad *pad)
{
   XYZPoint pos(0, 0, 0);

   std::array<Double_t, 512> floatADC = pad->GetADC();

   // Get baseline value.
   double baseline{};
   Double_t *maxAdcIt{nullptr};
   Double_t maxAdc{};
   Double_t charge{-999};
   Int_t maxAdcIdx;
   if (fPositivePolarity) {
      maxAdcIt = std::max_element(floatADC.begin() + fMinIndx, floatADC.end() - 12);
      maxAdcIdx = std::distance(floatADC.begin(), maxAdcIt);
      maxAdc = *maxAdcIt; // copy value to avoid changing floatADC
   } else {
      maxAdcIt = std::min_element(floatADC.begin() + fMinIndx, floatADC.end() - 12);
      maxAdcIdx = std::distance(floatADC.begin(), maxAdcIt);
      maxAdc = *maxAdcIt; // copy value to avoid changing floatADC
   }

   float count = 0;
   for (int i = std::max(fMinIndx, maxAdcIdx + fLowBLRegion[0]); i < std::min(maxAdcIdx + fLowBLRegion[1], 500);
        i++) { // more baseline added
      baseline += floatADC[i];
      count += 1;
   }
   for (int i = std::max(fMinIndx, maxAdcIdx + fHighBLRegion[0]); i < std::min(maxAdcIdx + fHighBLRegion[1], 500);
        i++) { // more baseline added
      baseline += floatADC[i];
      count += 1;
   }

   baseline /= count;
   LOG(debug) << "Baseline calculated: " << baseline;

   if (fDumpTraces) {
      std::ofstream ofile("traces_si.dat", std::ios::app);
      if (abs(maxAdc - baseline) > fThreshold) {
         ofile << pad->GetPadNum() << "  ";
         for (int i = 0; i < 512; ++i) {
            ofile << floatADC[i] << "  ";
         }
         ofile << std::endl;
      }
      ofile.close();
   }

   charge = maxAdc - baseline;

   if (!fPositivePolarity) {
      charge *= -1;
   }

   if (!shouldSaveHit(charge, fThreshold, maxAdcIdx))
      return {};

   LOG(debug) << "========== float ADC array max value ========== " << *maxAdcIt;
   LOG(debug) << "========== float ADC array max index: " << maxAdcIdx << " ==========";

   // Calculation of the mean value of the peak time by interpolating the pulse
   Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                      (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);
   Double_t TBCorr = getTBCorr(floatADC, maxAdcIdx);
   Double_t QHitTot = std::abs(std::accumulate(floatADC.begin() + maxAdcIdx + fEnergyIntegral[0],
                                               floatADC.begin() + maxAdcIdx + fEnergyIntegral[1], 0) /
                                  (fEnergyIntegral[1] - fEnergyIntegral[0]) -
                               baseline);

   auto hit = std::make_unique<AtHit>(0, pos, charge);

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
   LOG(debug) << charge << "   " << threshold << "  " << tb;
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
   if (floatADCVector.size() >= 256) {
      for (int i = 0; i < 256; i++) {
         floatADC[i] = floatADCVector[i];
         if (fOverflowReconstruction) {
            if (i > 0) {
               if ((floatADC[i - 1] - floatADC[i]) > 1000) {
                  floatADC[i] += 4096; // overflow
               }
            }
         }
      }
   } else {
      LOG(error) << "There are not 256 ADC values in the GAGG trace. Skipping!";
      return {};
   }

   Double_t charge{-999};
   Double_t baseline{0};
   Int_t maxAdcIdx = -1;
   Double_t maxAdc = -1;
   int min_overflow = -1;
   int max_overflow = -1;

   for (int i = fMinIndx; i < floatADCVector.size() - 12; ++i) {
      if (floatADC[i] > maxAdc) {
         maxAdc = floatADC[i];
         maxAdcIdx = i;
      }
      if (floatADC[i] == 8191) { // overflow hack
         if (min_overflow == -1) {
            min_overflow = i;
         } else {
            max_overflow = i;
         }
      }
   }
   bool saturation = false;
   if (min_overflow > -1 && max_overflow > -1) {
      saturation = true;
      maxAdcIdx = (int)((min_overflow + max_overflow) / 2);
   }

   float prebaseline = 0;
   float postbaseline = 0;
   float count = 0;
   float precount = 0;
   float postcount = 0;
   for (int i = std::max(fMinIndx, maxAdcIdx + fLowBLRegion[0]);
        i < std::min(maxAdcIdx + fLowBLRegion[1], (int)(floatADCVector.size() - 12)); i++) { // more baseline added
      baseline += floatADC[i];
      prebaseline += floatADC[i];
      count += 1;
      precount += 1;
   }
   for (int i = std::max(fMinIndx, maxAdcIdx + fHighBLRegion[0]);
        i < std::min(maxAdcIdx + fHighBLRegion[1], (int)(floatADCVector.size() - 12)); i++) { // more baseline added
      baseline += floatADC[i];
      postbaseline += floatADC[i];
      count += 1;
      postcount += 1;
   }
   if (count > 0) {
      baseline /= count;
   }
   if (precount > 0) {
      prebaseline /= precount;
   }
   if (postcount > 0) {
      postbaseline /= postcount;
   }
   LOG(debug) << "Baseline calculated: " << baseline;
   charge = maxAdc - baseline;

   if (fDumpTraces) {
      std::cout << " Max ADC = " << maxAdc << std::endl;
      std::cout << "Baseline calculated: " << baseline;
      std::cout << " Diff ADC = " << maxAdc - baseline << std::endl;

      std::ofstream ofile("traces_gagg.dat", std::ios::app);
      if (abs(maxAdc - baseline) > fThreshold) {
         ofile << genTrace->GetName() << "  " << genTrace->GetTraceID() << "  ";
         for (int i = 0; i < 256; ++i) {
            ofile << floatADC[i] << "  ";
         }
         ofile << std::endl;
      }
      ofile.close();
   }

   if (fDumpTraces && saturation) {
      std::ofstream ofile("traces_gagg_saturated.dat", std::ios::app);
      if (abs(maxAdc - baseline) > fThreshold) {
         ofile << genTrace->GetName() << "  " << genTrace->GetTraceID() << "  ";
         for (int i = 0; i < 256; ++i) {
            ofile << floatADC[i] << "  ";
         }
         ofile << std::endl;
      }
      ofile.close();
   }

   if (!shouldSaveHit(charge, fThreshold, maxAdcIdx)) {
      LOG(debug) << "GAGG trace did not pass threshold.";
      return {};
   }

   if (saturation && fFitClipped) { // fit the good part of the trace to extrapolate the peak
      // using a TGraph and a TF1 now because it's an easy interaface. May well be a more efficent way to go about this
      TGraph gr;
      int n = 0;
      for (int i = std::max(fMinIndx, min_overflow - 4);
           i < std::min(max_overflow + 7, (int)(floatADCVector.size() - 12)); i++) { // more baseline added
         if (floatADC[i] < 8191) {                                                   // overflow hack
            gr.SetPoint(n, i, floatADC[i]);
            ++n;
         }
      }
      // params are
      //[0] : baseline
      //[1] : amplitude
      //[2] : mean
      //[3] : sigma
      //[4] : skewness
      //
      std::cout << "saturated GAGG with " << n << " points to fit" << std::endl;

      TF1 func(
         "func",
         [&](double *x, double *p) {
            return p[0] + p[1] * p[4] * exp(p[4] / 2. * (2. * p[2] + p[4] * p[3] * p[3] - 2. * x[0])) *
                             TMath::Erfc((p[2] + p[4] * p[3] * p[3] - x[0]) / (sqrt(2) * p[3]));
         },
         0, 256, 5);
      func.FixParameter(0, prebaseline);
      func.SetParameter(1, 10000);
      func.SetParLimits(1, 0, 500000);
      func.SetParameter(2, maxAdcIdx - 2);
      func.SetParLimits(2, min_overflow - 4, max_overflow + 4);
      func.FixParameter(3, 2.8);
      func.FixParameter(4, 0.15);
      gr.Fit(&func, "R");
      func.ReleaseParameter(3);
      func.SetParLimits(3, 2.0, 4.0);
      func.ReleaseParameter(4);
      func.SetParLimits(4, 0.05, 0.25);
      gr.Fit(&func, "", "", 0, 256);
      std::cout << "saturated GAGG fit to " << func.GetParameter(1) << "  " << func.GetParameter(3) << "  "
                << func.GetParameter(4) << "  " << func.GetMaximum() << std::endl;
      if (fDumpTraces) {
         std::ofstream ofile("traces_gagg_fits.dat", std::ios::app);
         if (abs(maxAdc - baseline) > fThreshold) {
            ofile << genTrace->GetName() << "  " << genTrace->GetTraceID() << "  ";
            for (int i = 0; i < 256; ++i) {
               ofile << func.Eval(i) << "  ";
            }
            ofile << std::endl;
         }
         ofile.close();
      }
      // resample
      for (int i = min_overflow; i <= max_overflow; ++i) {
         floatADC[i] = func.Eval(i);
         if (floatADC[i] > maxAdc) {
            maxAdc = floatADC[i];
            maxAdcIdx = i;
         }
      }
   }

   if (saturation && fDumpTraces) {
      std::ofstream ofile("traces_gagg_recon.dat", std::ios::app);
      if (abs(maxAdc - baseline) > 50) {
         ofile << genTrace->GetName() << "  " << genTrace->GetTraceID() << "  ";
         for (int i = 0; i < 256; ++i) {
            ofile << floatADC[i] << "  ";
         }
         ofile << std::endl;
      }
      ofile.close();
   }
   // Calculation of the mean value of the peak time by interpolating the pulse
   Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                      (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);
   Double_t QHitTot = std::abs(std::accumulate(floatADC.begin() + maxAdcIdx + fEnergyIntegral[0],
                                               floatADC.begin() + maxAdcIdx + fEnergyIntegral[1], 0) /
                                  (fEnergyIntegral[1] - fEnergyIntegral[0]) -
                               baseline);

   auto hit = std::make_unique<AtHit>(0, pos, charge);

   hit->SetTimeStamp(maxAdcIdx);
   hit->SetTimeStampCorrInter(timemax);
   hit->SetTraceIntegral(QHitTot);

   HitVector ret;
   ret.push_back(std::move(hit));
   return ret;
}

ClassImp(AtPSASi);
