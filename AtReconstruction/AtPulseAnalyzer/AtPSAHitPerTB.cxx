#include "AtPSAHitPerTB.h"

#include <FairLogger.h>

// AtTPCROOT classes
#include "AtEvent.h"
#include "AtHit.h"
#include "AtRawEvent.h"

// STL
#include "AtPad.h" // for AtPad

#include <Math/Point2D.h> // for PositionVector2D
#include <TVector3.h>     // for TVector3

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

void AtPSAHitPerTB::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{
   Int_t numPads = rawEvent->GetNumPads();
   Double_t QEventTot = 0.0;
   Double_t RhoVariance = 0.0;
   std::map<Int_t, Int_t> PadMultiplicity;
   Float_t mesh[512] = {0};

   Int_t iPad = 0;

   //#pragma omp parallel for ordered schedule(dynamic,1) private(iPad)
   for (iPad = 0; iPad < numPads; iPad++) {

      const AtPad *pad = rawEvent->GetPads().at(iPad).get();
      Int_t PadNum = pad->GetPadNum();
      Int_t PadHitNum = 0;

      auto pos = pad->GetPadCoord();
      Double_t zPos = 0;
      Double_t charge = 0;

      if (!(pad->IsPedestalSubtracted())) {
         LOG(ERROR) << "Pedestal should be subtracted to use this class!";

         // return;
      }

      auto adc = pad->GetADC();
      Double_t floatADC[512] = {0};

      for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {
         floatADC[iTb] = adc[iTb];

         if (floatADC[iTb] > fThreshold) {

            mesh[iTb] += floatADC[iTb];
            if (iTb > fIniTB && iTb < fEndTB)
               QEventTot +=
                  floatADC[iTb]; // This allows to constrain the calculation of the charge avoiding noisy timebuckets
            zPos = CalculateZGeo(iTb);
            charge = adc[iTb];

            auto &hit = event->AddHit(PadNum, XYZPoint(pos.X(), pos.Y(), zPos), charge);
            PadHitNum++;
            hit.SetTimeStamp(iTb);
            if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
               std::cout << " AtPSAProtoFull::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum()
                         << std::endl;
         } // if Threshold
      }    // For iTb

      PadMultiplicity.insert(std::pair<Int_t, Int_t>(PadNum, PadHitNum));

   } // Pad loop

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      event->SetMeshSignal(iTb, mesh[iTb]);
   event->SetMultiplicityMap(PadMultiplicity);
   event->SetEventCharge(QEventTot);
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
