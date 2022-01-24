#include "AtPSASimple2.h"

#include "FairLogger.h"

// AtTPCROOT classes
#include "AtCalibration.h"
#include "AtRawEvent.h"
#include "AtEvent.h"
#include "AtHit.h"

// ROOT classes
#include "TH1F.h"
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"
#include "TSpectrum.h"

// STL
#include <algorithm>
#include <cmath>
#include <map>

#ifdef _OPENMP
#include <omp.h>
#endif

ClassImp(AtPSASimple2) AtPSASimple2::AtPSASimple2()
{
   fBackGroundSuppression = kFALSE;
   fBackGroundInterp = kFALSE;
   fIsPeakFinder = kFALSE;
   fIsMaxFinder = kFALSE;
   fIsBaseCorr = kFALSE;
   fIsTimeCorr = kFALSE;
}

AtPSASimple2::~AtPSASimple2() {}

void AtPSASimple2::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{

   Int_t numPads = rawEvent->GetNumPads();
   Int_t hitNum = 0;
   Double_t QEventTot = 0.0;
   Double_t RhoVariance = 0.0;
   Double_t RhoMean = 0.0;
   Double_t Rho2 = 0.0;
   std::map<Int_t, Int_t> PadMultiplicity;
   Float_t mesh[512] = {0};

   auto mcPointsMap = rawEvent->GetSimMCPointMap();
   LOG(info) << "MC Simulated points Map size " << mcPointsMap.size();

   /* for (const auto& entry : mcPointsMap)
      {
      std::cout<<entry.first<<"  "<<entry.second<<"\n";
      } */

   //#pragma omp parallel for ordered schedule(dynamic,1) private(iPad)
   for (auto &pad : rawEvent->GetPads()) {

      Int_t PadNum = pad.GetPadNum();
      Int_t pSizeID = pad.GetSizeID();
      Double_t gthreshold = -1;
      if (pSizeID == 0)
         gthreshold = fThresholdlow; // threshold for central pads
      else if (pSizeID == 1)
         gthreshold = fThreshold; // threshold for big pads
      else
         gthreshold = fThreshold; // default threshold for all pads

      Double_t QHitTot = 0.0;
      Int_t PadHitNum = 0;
      TVector3 HitPos;
      TVector3 HitPosRot;
      Bool_t fValidBuff = kTRUE;
      Bool_t fValidThreshold = kTRUE;
      Bool_t fValidDerivative = kTRUE;
      TRotation r;
      TRotation ry;
      TRotation rx;
      r.RotateZ(272.0 * TMath::Pi() / 180.0);
      ry.RotateY(180.0 * TMath::Pi() / 180.0);
      rx.RotateX(6.0 * TMath::Pi() / 180.0);

      Double_t xPos = pad.GetPadXCoord();
      Double_t yPos = pad.GetPadYCoord();
      Double_t zPos = 0;
      Double_t xPosRot = 0;
      Double_t yPosRot = 0;
      Double_t zPosRot = 0;
      Double_t xPosCorr = 0;
      Double_t yPosCorr = 0;
      Double_t zPosCorr = 0;
      Double_t charge = 0;
      Int_t maxAdcIdx = 0;
      Int_t numPeaks = 0;

      if ((xPos < -9000 || yPos < -9000) && !pad.IsAux()) {
         // std::cout<<" Is Auxiliary? "<<pad.IsAux()<<" Pad Num "<<PadNum<<"\n";
         continue; // Skip invalid pads that are not
      } else if (pad.IsAux()) {

         // std::cout<<" Is Auxiliary 2? "<<pad.IsAux()<<" Pad Num "<<PadNum<<"\n";
         event->AddAuxPad(&pad);
         continue;
      }

      CalcLorentzVector();

      if (!(pad.IsPedestalSubtracted())) {
         LOG(ERROR) << "Pedestal should be subtracted to use this class!";
      }

      Double_t *adc = pad.GetADC();
      Double_t floatADC[512] = {0};
      Double_t dummy[512] = {0};
      Double_t bg[512] = {0};

      if (fCalibration->IsGainFile()) {
         adc = fCalibration->CalibrateGain(adc, PadNum);
      }
      if (fCalibration->IsJitterFile()) {
         adc = fCalibration->CalibrateJitter(adc, PadNum);
      }

      for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {
         floatADC[iTb] = adc[iTb];
         QHitTot += adc[iTb];
         bg[iTb] = adc[iTb];
         // if(floatADC[iTb]!=0)std::cout << "Pad  "<<iPad<<"  tb  "<<iTb<<"  ADC  "<<floatADC[iTb] << '\n';
      }

      TSpectrum *PeakFinder = new TSpectrum;
      if (fIsPeakFinder)
         numPeaks = PeakFinder->SearchHighRes(floatADC, dummy, fNumTbs, 4.7, 5, fBackGroundSuppression, 3, kTRUE, 3);
      if (fIsMaxFinder)
         numPeaks = 1;

      TSpectrum *BGInter = new TSpectrum;
      if (fBackGroundInterp) {
         BGInter->Background(bg, fNumTbs, 6, TSpectrum::kBackDecreasingWindow, TSpectrum::kBackOrder2, kTRUE,
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
                  // if(PadNum==9788) std::cout<<" Time Bucket "<<i<<" floatADC "<<floatADC[i]<<std::endl;

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

            if (fIsMaxFinder) {
               if ((gthreshold > 0 && charge < gthreshold) || maxTime < 20 ||
                   maxTime > 500) // TODO: Does this work when the polarity is negative??
                  fValidThreshold = kFALSE;
            }
            if (fIsPeakFinder) {
               if (gthreshold > 0 && charge < gthreshold)
                  fValidThreshold = kFALSE;
            }

            // if (zPos > 0 || zPos < -fMaxDriftLength)
            // if (zPos < 0 || zPos > fMaxDriftLength)
            // continue;
            /*   if(PadNum== 9611){
               std::cout<<" PadNum "<<PadNum<<" Charge "<<charge<<" maxTime "<<maxTime<<std::endl;
               std::cout<<" Valid Threshold : "<<fValidThreshold<<std::endl;
               } */

            if (fValidThreshold && fValidDerivative) {

               // Sum only if Hit is valid - We only sum once (iPeak==0) to account for the
               // whole spectrum.
               if (iPeak == 0)
                  QEventTot += QHitTot;

               TVector3 posRot = RotateDetector(xPos, yPos, zPos, maxAdcIdx);

               AtHit *hit = new AtHit(PadNum, hitNum, xPos, yPos, zPos, charge);
               hit->SetPositionCorr(posRot.X(), posRot.Y(), posRot.Z());
               hit->SetTimeStamp(maxAdcIdx);
               hit->SetTimeStampCorr(TBCorr);
               hit->SetTimeStampCorrInter(timemax);
               hit->SetBaseCorr(basecorr / 10.0);
               hit->SetSlopeCnt(slope_cnt);
               PadHitNum++;
               hit->SetQHit(QHitTot);
               // TODO: The charge of each hit is the total charge of the spectrum, so for double
               // structures this is unrealistic.

               HitPos = hit->GetPosition();
               Rho2 += HitPos.Mag2();
               RhoMean += HitPos.Mag();
               if ((xPos < -9000 || yPos < -9000) && pad.GetPadNum() != -1)
                  std::cout << " AtPSASimple2::Analysis Warning! Wrong Coordinates for Pad : " << pad.GetPadNum()
                            << std::endl;

               // Tracking MC points
               if (mcPointsMap.size() > 0)
                  TrackMCPoints(mcPointsMap, hit);

               //#pragma omp ordered
               event->AddHit(hit);
               delete hit;

               hitNum++;

               for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  mesh[iTb] += floatADC[iTb];

            } // Valid Threshold
         }    // Peak Loop

         //    #pragma omp ordered
         // if(fValidThreshold && fValidBuff){
         // PadMultiplicity.insert(std::pair<Int_t,Int_t>(PadNum,PadHitNum));std::cout<<" PadNum : "<<PadNum<<"
         // PadHitNum : "<<PadHitNum<<std::endl;}
         //#pragma omp ordered
         PadMultiplicity.insert(std::pair<Int_t, Int_t>(PadNum, PadHitNum));

      } // if Valid Num Peaks

      delete PeakFinder;
      delete BGInter;

   } // Pad Loop

   RhoVariance = Rho2 - (pow(RhoMean, 2) / (event->GetNumHits()));
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
