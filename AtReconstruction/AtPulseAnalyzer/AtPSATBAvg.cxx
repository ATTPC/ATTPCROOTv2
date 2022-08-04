#include "AtPSATBAvg.h"

#include "AtHit.h"
#include "AtPad.h"

#include <FairLogger.h>

#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint

#include <algorithm>
#include <array>  // for array
#include <memory> // for unique_ptr, make_unique
#include <numeric>
#include <utility> // for pair

//#ifdef _OPENMP
//#include <omp.h>
//#endif
using XYZPoint = ROOT::Math::XYZPoint;

ClassImp(AtPSATBAvg);

AtPSATBAvg::HitVector AtPSATBAvg::AnalyzePad(AtPad *pad)
{
   LOG(debug) << "Running PSA on pad " << pad->GetPadNum();
   Int_t PadNum = pad->GetPadNum();

   XYZPoint pos(pad->GetPadCoord().X(), pad->GetPadCoord().Y(), 0);

   if (pos.X() < -9000 || pos.Y() < -9000) {
      LOG(debug) << "Skipping pad, position is invalid";
      return {};
   }

   if (!(pad->IsPedestalSubtracted())) {
      LOG(ERROR) << "Pedestal should be subtracted to use this class!";
   }

   std::array<Double_t, 512> floatADC = pad->GetADC();

   // Skip pad if we think it is saturated
   if (*std::max_element(floatADC.begin(), floatADC.end()) > fMaxThreshold)
      return {};

   Double_t traceIntegral = std::accumulate(floatADC.begin(), floatADC.end(), 0);

   HitVector hits;
   for (int i = 0; i < 512 / fTBtoAvg; ++i) {
      Int_t idx = i * fTBtoAvg;
      Double_t charge = std::accumulate(floatADC.begin() + idx, floatADC.begin() + idx + fTBtoAvg, 0);
      charge /= fTBtoAvg;

      if (charge < getThreshold(pad->GetSizeID()))
         continue;

      Double_t zTB = idx + static_cast<double>(fTBtoAvg) / 2;
      pos.SetZ(CalculateZGeo(zTB));

      auto hit = std::make_unique<AtHit>(PadNum, pos, charge);

      hit->SetTimeStamp(zTB);
      hit->SetTimeStampCorr(zTB);
      hit->SetTimeStampCorrInter(zTB);
      hit->SetTraceIntegral(traceIntegral);
      hits.push_back(std::move(hit));
   }
   return hits;
}
