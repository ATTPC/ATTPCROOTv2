#include "AtDigiPar.h"

#include <FairLogger.h>
#include <FairParGenericSet.h>
#include <FairParamList.h>

#include <TString.h>
#include <TSystem.h>

#include <iostream>
#include <memory>
#include <string>

ClassImp(AtDigiPar);

AtDigiPar::AtDigiPar(const Char_t *name, const Char_t *title, const Char_t *context)
   : FairParGenericSet("AtDigiPar", "AtTPC Parameter Container", ""), fInitialized(kFALSE)
{
}

/// returns the time duration of a time bucket in given sampling time in ns.
Int_t AtDigiPar::GetTBTime() const
{

   switch (fSamplingRate) {
   case 3: return 320;
   case 6: return 160;
   case 12: return 80;
   case 25: return 40;
   case 50: return 20;
   case 100: return 10;
   default: return -1;
   }
}

Bool_t AtDigiPar::getParams(FairParamList *paramList) // TODO Change all these parameters
{
   if (!paramList) {
      LOG(fatal) << "Parameter list doesn't exist!";
      return kFALSE;
   }

   if (!fInitialized) {

      if (!(paramList->fill("BField", &fBField))) {
         LOG(fatal) << "Cannot find BField parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("EField", &fEField))) {
         LOG(fatal) << "Cannot find EField parameter!";
         return kFALSE;
      }

      if (!(paramList->fill("TBEntrance", &fTBEntrance))) {
         LOG(fatal) << "Cannot find TBEntrance parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("ZPadPlane", &fZPadPlane))) {
         LOG(fatal) << "Cannot find ZPadPlane parameter!";
         return kFALSE;
      }

      if (!(paramList->fill("EIonize", &fEIonize))) {
         LOG(fatal) << "Cannot find EIonize parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Fano", &fFano))) {
         LOG(fatal) << "Cannot find Fano parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("CoefL", &fCoefL))) {
         LOG(fatal) << "Cannot find CoefL parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("CoefT", &fCoefT))) {
         LOG(fatal) << "Cannot find CoefT parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("GasPressure", &fGasPressure))) {
         LOG(fatal) << "Cannot find GasPressure parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Density", &fDensity))) {
         LOG(fatal) << "Cannot find Density parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("DriftVelocity", &fDriftVelocity))) {
         LOG(fatal) << "Cannot find Density parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Gain", &fGain))) {
         LOG(fatal) << "Cannot find Gain parameter!";
         return kFALSE;
      }

      if (!(paramList->fill("SamplingRate", &fSamplingRate))) {
         LOG(fatal) << "Cannot find GETGain parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("GETGain", &fGETGain))) {
         LOG(fatal) << "Cannot find GETGain parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("PeakingTime", &fPeakingTime))) {
         LOG(fatal) << "Cannot find PeakingTime parameter!";
         return kFALSE;
      }
   }

   return kTRUE;
}

void AtDigiPar::putParams(FairParamList *paramList)
{
   if (!paramList) {
      LOG(fatal) << "Parameter list doesn't exist!";
      return;
   }

   paramList->add("EField", fEField);
   paramList->add("BField", fBField);

   paramList->add("TBEntrance", fTBEntrance);
   paramList->add("ZPadPlane", fZPadPlane);

   paramList->add("EIonize", fEIonize);
   paramList->add("Fano", fFano);
   paramList->add("CoefL", fCoefL);
   paramList->add("CoefT", fCoefT);
   paramList->add("DriftVelocity", fDriftVelocity);
   paramList->add("Density", fDensity);
   paramList->add("GasPressure", fGasPressure);
   paramList->add("Gain", fGain);

   paramList->add("SamplingRate", fSamplingRate);
   paramList->add("GETGain", fGETGain);
   paramList->add("PeakingTime", fPeakingTime);
}
