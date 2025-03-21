#include "AtPSAHitPerTBInRegion.h"

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

using XYZPoint = ROOT::Math::XYZPoint;
AtPSAHitPerTBInRegion::HitVector AtPSAHitPerTBInRegion::AnalyzePad(AtPad *pad)
{
   auto pos = pad->GetPadCoord();
   if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
      LOG(error) << " AtPSAHitPErTBInRegion::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum()
                 << std::endl;

   if (!(pad->IsPedestalSubtracted())) {
      LOG(error) << "Pedestal should be subtracted to use this class!";
      // return;
   }

   HitVector hits;
   auto adc = pad->GetADC();
   for (Int_t iTb = fIniTB; iTb < fEndTB; iTb++) {

      // We are above threshold, so create a hit
      if (adc[iTb] > getThreshold(pad->GetSizeID())) {

         double TBOffset = fUniform(fRNG);

         auto hit = std::make_unique<AtHit>(pad->GetPadNum(), XYZPoint(pos.X(), pos.Y(), CalculateZGeo(iTb + TBOffset)),
                                            adc[iTb]);
         hit->SetTimeStamp(iTb + TBOffset);
         hit->SetTraceIntegral(adc[iTb]);
         hits.push_back(std::move(hit));
      } // if Threshold
   }

   return hits;
}

void AtPSAHitPerTBInRegion::SetTBLimits(std::pair<Int_t, Int_t> limits)
{
   if (limits.first >= limits.second) {
      std::cout << " Warning AtPSAHitPerTBInRegion::SetTBLimits -  Wrong Time Bucket limits. Setting default limits "
                   "(0,512) ... "
                << "\n";
      fIniTB = 0;
      fEndTB = 512;

   } else {
      if (limits.first < 0)
         fIniTB = 0;
      else
         fIniTB = limits.first;

      if (limits.second > fNumTbs)
         fEndTB = fNumTbs;
      else
         fEndTB = limits.second;
   }
}

ClassImp(AtPSAHitPerTBInRegion)
