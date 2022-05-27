#include "AtPSATBAvg.h"

#include "AtEvent.h"
#include "AtHit.h"
#include "AtPad.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint

#include <algorithm>
#include <array> // for array
#include <cmath>
#include <map>
#include <memory> // for unique_ptr, make_unique
#include <numeric>
#include <utility> // for pair
#include <vector>  // for vector

//#ifdef _OPENMP
//#include <omp.h>
//#endif
using XYZPoint = ROOT::Math::XYZPoint;

ClassImp(AtPSATBAvg);
Double_t AtPSATBAvg::getThreshold(int padSize)
{
   if (padSize == 0)
      return fThresholdlow; // threshold for central pads
   else
      return fThreshold; // threshold for big pads (or all other not small)
}

void AtPSATBAvg::Analyze(AtRawEvent *rawEvent, AtEvent *event)
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

      // Skip pad if we think it is saturated
      if (*std::max_element(floatADC.begin(), floatADC.end()) > fMaxThreshold)
         continue;

      Double_t QHitTot = std::accumulate(floatADC.begin(), floatADC.end(), 0);

      for (int i = 0; i < 512 / fTBtoAvg; ++i) {
         Int_t idx = i * fTBtoAvg;
         Double_t charge = std::accumulate(floatADC.begin() + idx, floatADC.begin() + idx + fTBtoAvg, 0);
         charge /= fTBtoAvg;

         if (charge > getThreshold(pad->GetSizeID())) {

            Double_t zTB = idx + static_cast<double>(fTBtoAvg) / 2;
            pos.SetZ(CalculateZGeo(zTB));

            auto &hit = event->AddHit(PadNum, pos, charge);
            LOG(debug) << "Added hit with ID" << hit.GetHitID();

            hit.SetTimeStamp(zTB);
            hit.SetTimeStampCorr(zTB);
            hit.SetTimeStampCorrInter(zTB);
            hit.SetTraceIntegral(QHitTot);

            Rho2 += pos.Mag2();
            RhoMean += pos.Rho();

            if (i == 0)
               QEventTot += QHitTot;

            // Tracking MC points
            auto mcPointsMap = rawEvent->GetSimMCPointMap();
            if (mcPointsMap.size() > 0) {
               LOG(debug) << "MC Simulated points Map size " << mcPointsMap.size();
               TrackMCPoints(mcPointsMap, hit);
            }

         } // End valid threshold

      } // End loop over all hits

      for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
         mesh[iTb] += floatADC[iTb];

      PadMultiplicity.insert(std::pair<Int_t, Int_t>(pad->GetPadNum(), 1));
   }

   // RhoVariance = Rho2 - (pow(RhoMean, 2) / (event->GetNumHits()));
   RhoVariance = Rho2 - (event->GetNumHits() * pow((RhoMean / event->GetNumHits()), 2));

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      event->SetMeshSignal(iTb, mesh[iTb]);
   event->SortHitArrayTime();
   event->SetMultiplicityMap(PadMultiplicity);
   event->SetRhoVariance(RhoVariance);
   event->SetEventCharge(QEventTot);
}
