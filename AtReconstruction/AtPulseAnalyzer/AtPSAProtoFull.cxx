#include "AtPSAProtoFull.h"

#include <FairLogger.h>

#include <TH1.h>

// AtTPCROOT classes
#include "AtRawEvent.h"
#include "AtEvent.h"
#include "AtHit.h"

// STL
#include <cmath>
#include <map>

#ifdef _OPENMP
#include <omp.h>
#endif

AtPSAProtoFull::AtPSAProtoFull()
{
   // fPeakFinder = new TSpectrum();
   // HPeak = new TH1F("HPeak","HPeak",512,0,511);
}

AtPSAProtoFull::~AtPSAProtoFull() {}

void AtPSAProtoFull::Analyze(AtRawEvent *rawEvent, AtEvent *event)
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
      TVector3 HitPos;
      Bool_t fValidBuff = kTRUE;
      Bool_t fValidThreshold = kTRUE;

      auto pos = pad->GetPadCoord();
      Double_t zPos = 0;
      Double_t charge = 0;

      if (!(pad->IsPedestalSubtracted())) {
         LOG(ERROR) << "Pedestal should be subtracted to use this class!";

         // return;
      }

      auto adc = pad->GetADC();
      Double_t floatADC[512] = {0};
      Double_t dummy[512] = {0};
      Int_t zeroPeak = 0;

      for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {
         floatADC[iTb] = adc[iTb];

         if (floatADC[iTb] > fThreshold) {

            mesh[iTb] += floatADC[iTb];
            if (iTb > fIniTB && iTb < fEndTB)
               QEventTot +=
                  floatADC[iTb]; // This allows to constrain the calculation of the charge avoiding noisy timebuckets
            zPos = CalculateZGeo(iTb);
            charge = adc[iTb];

            auto hit = event->AddHit(PadNum, XYZPoint(pos.X(), pos.Y(), zPos), charge);
            PadHitNum++;
            hit.SetTimeStamp(iTb);
            if ((pos.X() < -9000 || pos.Y() < -9000) && pad->GetPadNum() != -1)
               std::cout << " AtPSAProtoFull::Analysis Warning! Wrong Coordinates for Pad : " << pad->GetPadNum()
                         << std::endl;
            // std::cout<<"  =============== Next Hit Variance Info  =============== "<<std::endl;
            // std::cout<<" Hit Num : "<<hitNum<<"  - Hit Pos Rho2 : "<<HitPos.Mag2()<<"  - Hit Pos Rho :
            // "<<HitPos.Mag()<<std::endl; std::cout<<" Hit Coordinates : "<<pos.X()<<"  -  "<<pos.Y()<<" - "<<zPos<<" -
            // "<<std::endl; std::cout<<" Is Pad"<<pad->GetPadNum()<<" Valid? "<<pad->GetValidPad()<<std::endl;

         } // if Threshold
      }    // For iTb

      PadMultiplicity.insert(std::pair<Int_t, Int_t>(PadNum, PadHitNum));

   } // Pad loop

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      event->SetMeshSignal(iTb, mesh[iTb]);
   event->SetMultiplicityMap(PadMultiplicity);
   event->SetEventCharge(QEventTot);
}

ClassImp(AtPSAProtoFull)
