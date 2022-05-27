#include "AtPSAFull.h"

#include "AtEvent.h"
#include "AtHit.h"
#include "AtPad.h" // for AtPad
#include "AtRawEvent.h"

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
void AtPSAFull::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{
   Int_t numPads = rawEvent->GetNumPads();
   std::map<Int_t, Int_t> PadMultiplicity;
   Float_t mesh[512] = {0};

   for (auto iPad = 0; iPad < numPads; iPad++) {

      const AtPad *pad = rawEvent->GetPads().at(iPad).get();
      Int_t PadNum = pad->GetPadNum();
      Int_t PadHitNum = 0;
      TVector3 HitPos;

      Double_t charge = 0;
      Int_t maxAdcIdx = 0;
      Int_t numPeaks = 0;

      if (!(pad->IsPedestalSubtracted())) {
         LOG(ERROR) << "Pedestal should be subtracted to use this class!";
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

         mesh[iTb] += floatADC[iTb];
         if (floatADC[iTb] > fThreshold) {
            if (iTb == 0)
               initial = 0;
            else if (floatADC[iTb - 1] < fThreshold)
               initial = iTb;
            if (iTb == (fNumTbs - 1)) {
               final = iTb;
               if (final - initial > divider)
                  size = final - initial;
               interval.insert(std::pair<Int_t, Int_t>(initial, final));
            } else if (floatADC[iTb + 1] < fThreshold) {
               final = iTb;
               if (final - initial > divider)
                  size = final - initial;
               interval.insert(std::pair<Int_t, Int_t>(initial, final));
            }
         }
      }

      if (size < divider) {
         double zPos = CalculateZGeo(maxTime);
         auto pos = pad->GetPadCoord();
         if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
            std::cout << " AtPSAFull::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum() << std::endl;

         auto &hit = event->AddHit(PadNum, XYZPoint(pos.X(), pos.Y(), zPos), charge);
         hit.SetTimeStamp(maxTime);

         PadMultiplicity.insert(std::pair<Int_t, Int_t>(PadNum, hit.GetHitID()));
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

               auto &hit = event->AddHit(PadNum, XYZPoint(pos.X(), pos.Y(), zPos), charge);
               hit.SetTimeStamp(initInterval);
               charge = 0;

               PadMultiplicity.insert(std::pair<Int_t, Int_t>(PadNum, hit.GetHitID()));
            }
            ite++;
         }
         interval.clear();
      }
   } // Pad Loop

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      event->SetMeshSignal(iTb, mesh[iTb]);

   event->SortHitArrayTime();
   event->SetMultiplicityMap(PadMultiplicity);
}

ClassImp(AtPSAFull)
