// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>

#include "ATPhiRecoTask.hh"

ClassImp(ATPhiRecoTask);

ATPhiRecoTask::ATPhiRecoTask()
{
  fLogger = FairLogger::GetLogger();
  fPar = NULL;

  fIsPersistence = kFALSE;
  fPhiRecoMode = 0; // Default
  
  fPEventArray = new TClonesArray("ATProtoEvent"); 
}

ATPhiRecoTask::~ATPhiRecoTask()
{
}


void ATPhiRecoTask::SetPersistence(Bool_t value)     { fIsPersistence = value; }
void ATPhiRecoTask::SetThreshold(Double_t threshold) { fThreshold = threshold; }
void ATPhiRecoTask::SetPhiRecoMode(Int_t value)      { fPhiRecoMode = value; }

InitStatus
ATPhiRecoTask::Init()
{

  

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

    if (fPhiRecoMode == 0) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using ATPhiRecoSimple!");

    fPhiReco = new ATPhiRecoSimple();
  } else if (fPhiRecoMode == 1) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using ATPhiRecoTriple!");

    fPhiReco = new ATPhiRecoTriple();
  }

   //fPSA -> SetThreshold((Int_t)fThreshold);

   ioMan -> Register("ATProtoEvent", "ATTPC", fPEventArray, fIsPersistence);

   

  return kSUCCESS;
}

void
ATPhiRecoTask::SetParContainers()
{

  
  
  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");
}

void
ATPhiRecoTask::Exec(Option_t *opt)
{
   fPEventArray -> Delete();

 

  if (fEventHArray -> GetEntriesFast() == 0)
     return;

     ATEvent *event = (ATEvent *) fEventHArray -> At(0);
    
     //std::cout << "  Event Number :  " << Event -> GetEventID() << std::endl;
     ATProtoEvent *protoevent = (ATProtoEvent *) new ((*fPEventArray)[0]) ATProtoEvent();
     protoevent->SetEventID(event->GetEventID());
     fPhiReco->PhiAnalyze(event,protoevent);

  
}
