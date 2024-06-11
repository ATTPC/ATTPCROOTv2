#include "AtPSAFull.h"

#include "AtHit.h"
#include "AtPad.h" // for AtPad

#include <FairLogger.h> // for LOG

#include <Math/Point2D.h>    // for PositionVector2D
#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <TVector3.h>        // for TVector3

#include <array>    // for array
#include <iostream> // for basic_ostream::operator<<, operator<<
#include <map>
#include <memory>  // for allocator_traits<>::value_type
#include <utility> // for pair

/*
#ifdef _OPENMP
#include <omp.h>
#endif
*/
using XYZPoint = ROOT::Math::XYZPoint;

AtPSAFull::HitVector AtPSAFull::AnalyzePad(AtPad *pad)
{

   Int_t PadNum = pad->GetPadNum();
   Int_t PadHitNum = 0;
   TVector3 HitPos;

   Double_t charge = 0;
   Int_t maxAdcIdx = 0;
   Int_t numPeaks = 0;

   if (!(pad->IsPedestalSubtracted())) {
      LOG(error) << "Pedestal should be subtracted to use this class!";
      // return;
   }

   auto adc = pad->GetADC();
   Double_t floatADC[512] = {0};
   Double_t dummy[512] = {0};

   Int_t divider = 50;
   Int_t initial = 0;
   Int_t final = 0;
   Int_t endInterval = 0;
   Float_t max = 0.0;
   Int_t maxTime = 0;

   std::map<Int_t, Int_t> interval;

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      floatADC[iTb] = adc[iTb];

   for (Int_t ij = 20; ij < 500; ij++) { // Excluding first and last 12 Time Buckets
      if (floatADC[ij] > max) {
         max = floatADC[ij];
         maxTime = ij;
      }
   }

   Int_t size = 0;
   // maxAdcIdx = maxTime;

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {

      if (floatADC[iTb] > getThreshold(pad->GetSizeID())) {
         if (iTb == 0)
            initial = 0;
         else if (floatADC[iTb - 1] < getThreshold(pad->GetSizeID()))
            initial = iTb;
         if (iTb == (fNumTbs - 1)) {
            final = iTb;
            if (final - initial > divider)
               size = final - initial;
            interval.insert(std::pair<Int_t, Int_t>(initial, final));
         } else if (floatADC[iTb + 1] < getThreshold(pad->GetSizeID())) {
            final = iTb;
            if (final - initial > divider)
               size = final - initial;
            interval.insert(std::pair<Int_t, Int_t>(initial, final));
         }
      }
   }

   HitVector hits;
   if (size < divider) {
      double zPos = CalculateZGeo(maxTime);
      auto pos = pad->GetPadCoord();
      if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
         std::cout << " AtPSAFull::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum() << std::endl;

      auto hit = std::make_unique<AtHit>(PadNum, XYZPoint(pos.X(), pos.Y(), zPos), charge);
      hit->SetTimeStamp(maxTime);
      hits.push_back(std::move(hit));

   } else {
      auto ite = interval.begin();
      // Double_t *thePar = new Double_t[3];
      while (ite != interval.end()) {
         final = (ite->second);
         initial = (ite->first);
         Int_t reducedPoints = (final - initial) / divider;
         for (Int_t points = 0; points < reducedPoints + 1; points++) {
            Int_t initInterval = initial + points * divider;
            if (points == reducedPoints)
               endInterval = final;
            else
               endInterval = initial + ((points + 1) * divider) - 1;

            double zPos = CalculateZGeo(initInterval);
            auto pos = pad->GetPadCoord();
            if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
               std::cout << " AtPSAFull::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum()
                         << std::endl;

            for (Int_t iIn = initInterval; iIn < endInterval; iIn++)
               charge += floatADC[iIn] / divider; // reduced by divider!!!

            auto hit = std::make_unique<AtHit>(PadNum, XYZPoint(pos.X(), pos.Y(), zPos), charge);
            hit->SetTimeStamp(initInterval);
            charge = 0;
            hits.push_back(std::move(hit));
         }
         ite++;
      }
      interval.clear();
   }
   return hits;
}

ClassImp(AtPSAFull)
