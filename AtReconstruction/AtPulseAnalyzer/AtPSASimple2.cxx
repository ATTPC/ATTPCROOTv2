#include "AtPSASimple2.h"

#include "AtCalibration.h"
#include "AtEvent.h"
#include "AtHit.h"
#include "AtPad.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Rotation3D.h>
#include <TSpectrum.h>

#include <algorithm>
#include <array> // for array
#include <cmath>
#include <iostream> // for basic_ostream::operator<<
#include <map>
#include <memory>  // for unique_ptr, make_unique
#include <utility> // for pair
#include <vector>  // for vector

//#ifdef _OPENMP
//#include <omp.h>
//#endif
using XYZPoint = ROOT::Math::XYZPoint;
ClassImp(AtPSASimple2);

void AtPSASimple2::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{

   Int_t hitNum = 0;
   Double_t QEventTot = 0.0;
   Double_t RhoVariance = 0.0;
   Double_t RhoMean = 0.0;
   Double_t Rho2 = 0.0;
   std::map<Int_t, Int_t> PadMultiplicity;
   std::array<Float_t, 512> mesh{};
   mesh.fill(0);

   auto mcPointsMap = rawEvent->GetSimMCPointMap();
   LOG(debug) << "MC Simulated points Map size " << mcPointsMap.size();

   //#pragma omp parallel for ordered schedule(dynamic,1) private(iPad)
   for (const auto &pad : rawEvent->GetPads()) {

      LOG(debug) << "Running PSA on pad " << pad->GetPadNum();
      Int_t PadNum = pad->GetPadNum();
      Int_t pSizeID = pad->GetSizeID();
      Double_t gthreshold = -1;
      if (pSizeID == 0)
         gthreshold = fThresholdlow; // threshold for central pads
      else
         gthreshold = fThreshold; // threshold for big pads (or all other not small)

      Double_t QHitTot = 0.0;
      XYZPoint HitPos;
      ROOT::Math::Rotation3D HitPosRot;

      Bool_t fValidBuff = kTRUE;
      Bool_t fValidThreshold = kTRUE;
      Bool_t fValidDerivative = kTRUE;

      auto pos = pad->GetPadCoord();
      Double_t zPos = 0;
      Double_t charge = 0;
      Int_t maxAdcIdx = 0;
      Int_t numPeaks = 0;

      if (pos.X() < -9000 || pos.Y() < -9000) {
         LOG(debug) << "Skipping pad, position is invalid";
         continue;
      }

      if (!(pad->IsPedestalSubtracted())) {
         LOG(ERROR) << "Pedestal should be subtracted to use this class!";
      }

      auto adc = pad->GetADC();
      std::array<Double_t, 512> floatADC{};
      std::array<Double_t, 512> dummy{};
      std::array<Double_t, 512> bg{};
      floatADC.fill(0);
      dummy.fill(0);
      bg.fill(0);

      if (fCalibration.IsGainFile()) {
         adc = fCalibration.CalibrateGain(adc, PadNum);
      }
      if (fCalibration.IsJitterFile()) {
         adc = fCalibration.CalibrateJitter(adc, PadNum);
      }

      for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {
         floatADC[iTb] = adc[iTb];
         QHitTot += adc[iTb];
         bg[iTb] = adc[iTb];
      }

      auto PeakFinder = std::make_unique<TSpectrum>();
      if (fIsPeakFinder)
         numPeaks = PeakFinder->SearchHighRes(floatADC.data(), dummy.data(), fNumTbs, 4.7, 5, fBackGroundSuppression, 3,
                                              kTRUE, 3);
      if (fIsMaxFinder)
         numPeaks = 1;

      auto BGInter = std::make_unique<TSpectrum>();
      if (fBackGroundInterp) {
         BGInter->Background(bg.data(), fNumTbs, 6, TSpectrum::kBackDecreasingWindow, TSpectrum::kBackOrder2, kTRUE,
                             TSpectrum::kBackSmoothing7, kTRUE);
         for (Int_t iTb = 1; iTb < fNumTbs; iTb++) {
            floatADC[iTb] = floatADC[iTb] - bg[iTb];
            if (floatADC[iTb] < 0)
               floatADC[iTb] = 0;
         }
      }

      if (numPeaks == 0)
         fValidBuff = kFALSE;
      // continue;

      if (fValidBuff) {

         for (Int_t iPeak = 0; iPeak < numPeaks; iPeak++) {

            Float_t max = 0.0;
            Float_t min = 0.0;
            Int_t maxTime = 0;

            if (fIsPeakFinder) {
               maxAdcIdx = (Int_t)(ceil((PeakFinder->GetPositionX())[iPeak]));
               if (maxAdcIdx < 3 || maxAdcIdx > 509)
                  continue; // excluding the first and last 3 tb
            }
            //  Int_t maxAdcIdx = *std::max_element(floatADC,floatADC+fNumTbs);

            if (fIsMaxFinder) {
               for (Int_t ij = 20; ij < 500; ij++) // Excluding first and last 12 Time Buckets
               {
                  if (floatADC[ij] > max) {
                     max = floatADC[ij];
                     maxTime = ij;
                  }
               }

               maxAdcIdx = maxTime;
            }
            // Charge Correction due to mesh induction (base line)

            Double_t basecorr = 0.0;
            Double_t slope = 0.0;
            Int_t slope_cnt = 0;

            if (maxAdcIdx > 20)
               for (Int_t i = 0; i < 10; i++) {

                  basecorr += floatADC[maxAdcIdx - 8 - i];

                  if (i < 5) {
                     slope = (floatADC[maxAdcIdx - i] - floatADC[maxAdcIdx - i - 1]); // Derivate for 5 Timebuckets
                     // if(slope<0 && floatADC[maxAdcIdx]<3500 && fIsBaseCorr && fIsMaxFinder)
                     // fValidDerivative = kFALSE; //3500 condition to avoid killing saturated pads
                     if (slope < 0 && fIsBaseCorr && fIsMaxFinder)
                        slope_cnt++;
                  }
               }
            // Calculation of the mean value of the peak time by interpolating the pulse

            Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                               (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);

            // Time Correction by Center of Gravity
            Double_t TBCorr = 0.0;
            Double_t TB_TotQ = 0.0;

            if (maxAdcIdx > 11) {
               for (Int_t i = 0; i < 11; i++) {

                  if (floatADC[maxAdcIdx - i + 10] > 0 && floatADC[maxAdcIdx - i + 10] < 4000) {
                     TBCorr += (floatADC[maxAdcIdx - i + 5] - basecorr / 10.0) *
                               (maxAdcIdx - i + 5); // Substract the baseline correction
                     TB_TotQ += floatADC[maxAdcIdx - i + 5] - basecorr / 10.0;
                  }
               }
            }
            TBCorr = TBCorr / TB_TotQ;

            if (fIsBaseCorr)
               charge = floatADC[maxAdcIdx] - basecorr / 10.0;
            else
               charge = floatADC[maxAdcIdx];

            if (fIsTimeCorr)
               zPos = CalculateZGeo(TBCorr);
            else
               zPos = CalculateZGeo(maxAdcIdx);

            if (gthreshold > 0 && charge < gthreshold) {
               fValidThreshold = false;
               LOG(debug) << "Invalid threshold with charge: " << charge << " and threshold: " << gthreshold;
            }
            if (fIsMaxFinder && (maxTime < 20 || maxTime > 500)) {
               fValidThreshold = kFALSE;
               LOG(debug) << "Peak is outside valid time window (20,500) TBs.";
            }

            if (fValidThreshold && fValidDerivative) {

               // Sum only if Hit is valid - We only sum once (iPeak==0) to account for the
               // whole spectrum.
               if (iPeak == 0)
                  QEventTot += QHitTot;

               auto &hit = event->AddHit(PadNum, XYZPoint(pos.X(), pos.Y(), zPos), charge);
               LOG(debug) << "Added hit with ID" << hit.GetHitID();

               hit.SetTimeStamp(maxAdcIdx);
               hit.SetTimeStampCorr(TBCorr);
               hit.SetTimeStampCorrInter(timemax);

               hit.SetTraceIntegral(QHitTot);
               // TODO: The charge of each hit is the total charge of the spectrum, so for double
               // structures this is unrealistic.

               HitPos = hit.GetPosition();
               Rho2 += HitPos.Mag2();
               RhoMean += HitPos.Rho();
               if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
                  std::cout << " AtPSASimple2::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum()
                            << std::endl;

               // Tracking MC points
               //if (mcPointsMap.size() > 0)
	       //TrackMCPoints(mcPointsMap, hit);

               for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  mesh[iTb] += floatADC[iTb];

		  } // Valid Threshold 
         }    // Peak Loop

         //    #pragma omp ordered
         // if(fValidThreshold && fValidBuff)
         // PadMultiplicity.insert(std::pair<Int_t,Int_t>(PadNum,PadHitNum));

         //#pragma omp ordered
         PadMultiplicity.insert(std::pair<Int_t, Int_t>(pad->GetPadNum(), 1));

      } // if Valid Num Peaks
   }    // Pad Loop

   // RhoVariance = Rho2 - (pow(RhoMean, 2) / (event->GetNumHits()));
   RhoVariance = Rho2 - (event->GetNumHits() * pow((RhoMean / event->GetNumHits()), 2));

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      event->SetMeshSignal(iTb, mesh[iTb]);
   event->SortHitArrayTime();
   event->SetMultiplicityMap(PadMultiplicity);
   event->SetRhoVariance(RhoVariance);
   event->SetEventCharge(QEventTot);
}

void AtPSASimple2::SetBaseCorrection(Bool_t value)
{
   fIsBaseCorr = value;
}

void AtPSASimple2::SetTimeCorrection(Bool_t value)
{
   fIsTimeCorr = value;
}

void AtPSASimple2::SetBackGroundSuppression()
{
   fBackGroundSuppression = kTRUE;
}

void AtPSASimple2::SetBackGroundInterpolation()
{
   fBackGroundInterp = kTRUE;
}

void AtPSASimple2::SetPeakFinder()
{
   fIsPeakFinder = kTRUE;
   fIsMaxFinder = kFALSE;
}

void AtPSASimple2::SetMaxFinder()
{
   fIsMaxFinder = kTRUE;
   fIsPeakFinder = kFALSE;
}
