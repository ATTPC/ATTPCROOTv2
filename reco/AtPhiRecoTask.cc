// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>

#include "AtPhiRecoTask.h"

ClassImp(AtPhiRecoTask);

AtPhiRecoTask::AtPhiRecoTask()
{
  fLogger = FairLogger::GetLogger();
  fPar = NULL;

  fIsPersistence = kFALSE;
  fPhiRecoMode = 0; // Default
  
  fPEventArray = new TClonesArray("AtProtoEvent"); 
}

AtPhiRecoTask::~AtPhiRecoTask()
{
}


void AtPhiRecoTask::SetPersistence(Bool_t value)     { fIsPersistence = value; }
void AtPhiRecoTask::SetThreshold(Double_t threshold) { fThreshold = threshold; }
void AtPhiRecoTask::SetPhiRecoMode(Int_t value)      { fPhiRecoMode = value; }

InitStatus
AtPhiRecoTask::Init()
{

  

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

    if (fPhiRecoMode == 0) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using AtPhiRecoSimple!");

    fPhiReco = new AtPhiRecoSimple();
  } else if (fPhiRecoMode == 1) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using AtPhiRecoTriple!");

    fPhiReco = new AtPhiRecoTriple();
  }

   //fPSA -> SetThreshold((Int_t)fThreshold);

   ioMan -> Register("AtProtoEvent", "AtTPC", fPEventArray, fIsPersistence);

   

  return kSUCCESS;
}

void
AtPhiRecoTask::SetParContainers()
{

  
  
  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (AtDigiPar *) db -> getContainer("AtDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "AtDigiPar not found!!");
}

void
AtPhiRecoTask::Exec(Option_t *opt)
{
   fPEventArray -> Delete();

 

  if (fEventHArray -> GetEntriesFast() == 0)
     return;

     AtEvent *event = (AtEvent *) fEventHArray -> At(0);
    
     //std::cout << "  Event Number :  " << Event -> GetEventID() << std::endl;
     AtProtoEvent *protoevent = (AtProtoEvent *) new ((*fPEventArray)[0]) AtProtoEvent();
     protoevent->SetEventID(event->GetEventID());
     fPhiReco->PhiAnalyze(event,protoevent);

  
}
