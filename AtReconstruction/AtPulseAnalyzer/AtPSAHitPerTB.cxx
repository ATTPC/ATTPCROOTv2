#include "AtPSAHitPerTB.h"

#include "AtHit.h"
#include "AtPad.h" // for AtPad

#include <FairLogger.h>

#include <Math/Point2D.h>    // for PositionVector2D
#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint

#include <array>    // for array
#include <iostream> // for basic_ostream::operator<<, operator<<
#include <memory>   // for allocator_traits<>::value_type
#include <utility>  // for pair
#include <vector>
/*
#ifdef _OPENMP
#include <omp.h>
#endif
*/
using XYZPoint = ROOT::Math::XYZPoint;
AtPSAHitPerTB::HitVector AtPSAHitPerTB::AnalyzePad(AtPad *pad)
{
   auto pos = pad->GetPadCoord();
   if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
      LOG(error) << " AtPSAHitPErTB::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum() << std::endl;

   if (!(pad->IsPedestalSubtracted())) {
      LOG(error) << "Pedestal should be subtracted to use this class!";
      // return;
   }

   HitVector hits;
   auto adc = pad->GetADC();
   double traceIntegral = 0;
   for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {

      // We are above threshold, so create a hit
      if (adc[iTb] > getThreshold(pad->GetSizeID())) {

         // This allows to constrain the calculation of the charge avoiding noisy timebuckets
         if (iTb > fIniTB && iTb < fEndTB)
            traceIntegral += adc[iTb];

         auto hit = std::make_unique<AtHit>(pad->GetPadNum(), XYZPoint(pos.X(), pos.Y(), CalculateZGeo(iTb)), adc[iTb]);
         hit->SetTimeStamp(iTb);
         hits.push_back(std::move(hit));
      } // if Threshold
   }

   // Loop through all hits and add traceIntegral
   for (auto &hit : hits)
      hit->SetTraceIntegral(traceIntegral);

   return hits;
}

void AtPSAHitPerTB::SetTBLimits(std::pair<Int_t, Int_t> limits)
{
   if (limits.first >= limits.second) {
      std::cout << " Warning AtPSA::SetTBLimits -  Wrong Time Bucket limits. Setting default limits (0,512) ... "
                << "\n";
      fIniTB = 0;
      fEndTB = 512;

   } else {
      fIniTB = limits.first;
      fEndTB = limits.second;
   }
}

ClassImp(AtPSAHitPerTB)
