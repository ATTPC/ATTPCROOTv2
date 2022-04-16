
#include "AtS800Task.h"

#include <FairLogger.h>
#include <FairRun.h>
#include <FairTask.h>

#include <memory>

class FairRuntimeDb;

ClassImp(AtS800Task);

void AtS800Task::SetPersistence(Bool_t value)
{
   kIsPersistence = value;
}

InitStatus AtS800Task::Init()
{

   /*
   if(fS800Alg==0)fS800Array = new TClonesArray("AtS800N::AtS800");
   else if(fS800Alg==1) fS800Array = new TClonesArray("AtS800Mod");
   else if(fS800Alg==2) fS800Array = new TClonesArray("AtMlesacMod");
   else if(fS800Alg==3) fS800Array = new TClonesArray("AtLmedsMod");
   else{
     fLogger -> Error(MESSAGE_ORIGIN, "Cannot find S800 algorithm!");
     return kERROR;
   }

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
     fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
     return kERROR;
   }

   fEventHArray = (TClonesArray *) ioMan -> GetObject("AtEventH");
   if (fEventHArray == 0) {
     fLogger -> Error(MESSAGE_ORIGIN, "Cannot find AtEvent array!");
     return kERROR;
   }


   ioMan -> Register("AtS800", "AtTPC", fS800Array, kIsPersistence);
   /*/

   return kSUCCESS;
}

void AtS800Task::SetParContainers()
{

   FairRun *run = FairRun::Instance();
   if (!run) {
      LOG(fatal) << "No analysis run!";
      return;
   }

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      LOG(fatal) << "No runtime database!";

   /*fPar = (AtDigiPar *) db -> getContainer("AtDigiPar");
   if (!fPar)
     fLogger -> Fatal(MESSAGE_ORIGIN, "AtDigiPar not found!!");
     */
}

void AtS800Task::Exec(Option_t *opt)
{

   /*

     fS800Array -> Delete();

     if (fEventHArray -> GetEntriesFast() == 0)
      return;

     fEvent  = (AtEvent *) fEventHArray -> At(0);

     if(fS800Alg==0){
       AtS800N::AtS800 *S800 = (AtS800N::AtS800 *) new ((*fS800Array)[0]) AtS800N::AtS800();
       S800 -> SetTiltAngle(fTiltAngle);
       S800->SetModelType(fS800Model);
       S800->SetDistanceThreshold(fS800Threshold);
       S800->SetMinHitsLine(fMinHitsLine);
       if(kIsFullMode) S800->CalcS800Full(fEvent);
       else S800->CalcS800(fEvent);
     }


     if(fS800Alg==1){
       AtS800Mod * Rantest = (AtS800Mod *) new ((*fS800Array)[0]) AtS800Mod();
       Rantest->SetDistanceThreshold(fS800Threshold);
       Rantest->SetMinHitsLine(fMinHitsLine);
       Rantest->SetNumItera(fNumItera);
       Rantest->SetRanSamMode(fRandSamplMode);
       Rantest->CalcS800Mod(fEvent);
     }

     if(fS800Alg==2){
       AtMlesacMod * Rantest = (AtMlesacMod *) new ((*fS800Array)[0]) AtMlesacMod();
       Rantest->SetDistanceThreshold(fS800Threshold);
       Rantest->SetMinHitsLine(fMinHitsLine);
       Rantest->SetNumItera(fNumItera);
       Rantest->SetRanSamMode(fRandSamplMode);
       Rantest->CalcMlesacMod(fEvent);
     }

     if(fS800Alg==3){
       AtLmedsMod * Rantest = (AtLmedsMod *) new ((*fS800Array)[0]) AtLmedsMod();
       Rantest->SetDistanceThreshold(fS800Threshold);
       Rantest->SetMinHitsLine(fMinHitsLine);
       Rantest->SetNumItera(fNumItera);
       Rantest->SetRanSamMode(fRandSamplMode);
       Rantest->CalcLmedsMod(fEvent);
     }
     */
}
