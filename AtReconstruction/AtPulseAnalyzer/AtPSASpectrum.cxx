#include "AtPSASpectrum.h"

#include "AtEvent.h"
#include "AtHit.h"
#include "AtPad.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <TSpectrum.h>

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
ClassImp(AtPSASpectrum);

void AtPSASpectrum::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{

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

      auto pos = pad->GetPadCoord();
      Double_t zPos = 0;
      Double_t xPosCorr = 0;
      Double_t yPosCorr = 0;
      Double_t zPosCorr = 0;
      Double_t charge = 0;
      // Int_t maxAdcIdx = 0;
      // Int_t numPeaks = 0;

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
      floatADC.fill(0);
      dummy.fill(0);

      for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {
         floatADC[iTb] = adc[iTb];
         QHitTot += adc[iTb];
      }

      auto PeakFinder = std::make_unique<TSpectrum>();
      auto numPeaks =
         PeakFinder->SearchHighRes(floatADC.data(), dummy.data(), fNumTbs, 4.7, 5, fBackGroundSuppression, 3, kTRUE, 3);

      if (fBackGroundInterp) {
         subtractBackground(floatADC);
      }

      if (numPeaks != 0) {
         PadMultiplicity.insert(std::pair<Int_t, Int_t>(pad->GetPadNum(), 1));
      }
      for (Int_t iPeak = 0; iPeak < numPeaks; iPeak++) {

         auto maxAdcIdx = (Int_t)(ceil((PeakFinder->GetPositionX())[iPeak]));
         if (maxAdcIdx < 3 || maxAdcIdx > 509)
            continue; // excluding the first and last 3 tb

         // Calculation of the mean value of the peak time by interpolating the pulse
         Double_t timemax = 0.5 * (floatADC[maxAdcIdx - 1] - floatADC[maxAdcIdx + 1]) /
                            (floatADC[maxAdcIdx - 1] + floatADC[maxAdcIdx + 1] - 2 * floatADC[maxAdcIdx]);

         Double_t TBCorr = calcTbCorrection(floatADC, maxAdcIdx);

         charge = floatADC[maxAdcIdx];

         if (fIsTimeCorr)
            zPos = CalculateZGeo(TBCorr);
         else
            zPos = CalculateZGeo(maxAdcIdx);

         if (gthreshold > 0 && charge < gthreshold)
            LOG(debug) << "Invalid threshold with charge: " << charge << " and threshold: " << gthreshold;
         else {

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

            auto HitPos = hit.GetPosition();
            Rho2 += HitPos.Mag2();
            RhoMean += HitPos.Rho();
            if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
               std::cout << " AtPSASpectrum::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum()
                         << std::endl;

            // Tracking MC points
            if (mcPointsMap.size() > 0)
               TrackMCPoints(mcPointsMap, hit);

            for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
               mesh[iTb] += floatADC[iTb];

         } // Valid Threshold
      }    // Peak Loop
   }       // Pad Loop

   // RhoVariance = Rho2 - (pow(RhoMean, 2) / (event->GetNumHits()));
   RhoVariance = Rho2 - (event->GetNumHits() * pow((RhoMean / event->GetNumHits()), 2));

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      event->SetMeshSignal(iTb, mesh[iTb]);
   event->SortHitArrayTime();
   event->SetMultiplicityMap(PadMultiplicity);
   event->SetRhoVariance(RhoVariance);
   event->SetEventCharge(QEventTot);
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
