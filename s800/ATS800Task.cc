
#include "ATS800Task.hh"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>

ClassImp(ATS800Task);

ATS800Task::ATS800Task()
{

  fLogger = FairLogger::GetLogger();

  kIsPersistence = kFALSE;
  kIsFullMode    = kFALSE;


}

ATS800Task::~ATS800Task()
{
}

void   ATS800Task::SetPersistence(Bool_t value)             { kIsPersistence   = value; }

InitStatus
ATS800Task::Init()
{


  /*
  if(fS800Alg==0)fS800Array = new TClonesArray("ATS800N::ATS800");
  else if(fS800Alg==1) fS800Array = new TClonesArray("ATS800Mod");
  else if(fS800Alg==2) fS800Array = new TClonesArray("ATMlesacMod");
  else if(fS800Alg==3) fS800Array = new TClonesArray("ATLmedsMod");
  else{
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find S800 algorithm!");
    return kERROR;
  }

  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
    return kERROR;
  }

  fEventHArray = (TClonesArray *) ioMan -> GetObject("ATEventH");
  if (fEventHArray == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find ATEvent array!");
    return kERROR;
  }


  ioMan -> Register("ATS800", "ATTPC", fS800Array, kIsPersistence);
  /*/




  return kSUCCESS;
}


void
ATS800Task::SetParContainers()
{


  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  /*fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");
    */
}

void
ATS800Task::Exec(Option_t *opt)
{

    /*

      fS800Array -> Delete();

      if (fEventHArray -> GetEntriesFast() == 0)
       return;

      fEvent  = (ATEvent *) fEventHArray -> At(0);

      if(fS800Alg==0){
        ATS800N::ATS800 *S800 = (ATS800N::ATS800 *) new ((*fS800Array)[0]) ATS800N::ATS800();
        S800 -> SetTiltAngle(fTiltAngle);
        S800->SetModelType(fS800Model);
        S800->SetDistanceThreshold(fS800Threshold);
        S800->SetMinHitsLine(fMinHitsLine);
        if(kIsFullMode) S800->CalcS800Full(fEvent);
        else S800->CalcS800(fEvent);
      }


      if(fS800Alg==1){
        ATS800Mod * Rantest = (ATS800Mod *) new ((*fS800Array)[0]) ATS800Mod();
        Rantest->SetDistanceThreshold(fS800Threshold);
        Rantest->SetMinHitsLine(fMinHitsLine);
        Rantest->SetNumItera(fNumItera);
        Rantest->SetRanSamMode(fRandSamplMode);
        Rantest->CalcS800Mod(fEvent);
      }

      if(fS800Alg==2){
        ATMlesacMod * Rantest = (ATMlesacMod *) new ((*fS800Array)[0]) ATMlesacMod();
        Rantest->SetDistanceThreshold(fS800Threshold);
        Rantest->SetMinHitsLine(fMinHitsLine);
        Rantest->SetNumItera(fNumItera);
        Rantest->SetRanSamMode(fRandSamplMode);
        Rantest->CalcMlesacMod(fEvent);
      }

      if(fS800Alg==3){
        ATLmedsMod * Rantest = (ATLmedsMod *) new ((*fS800Array)[0]) ATLmedsMod();
        Rantest->SetDistanceThreshold(fS800Threshold);
        Rantest->SetMinHitsLine(fMinHitsLine);
        Rantest->SetNumItera(fNumItera);
        Rantest->SetRanSamMode(fRandSamplMode);
        Rantest->CalcLmedsMod(fEvent);
      }
      */

}
