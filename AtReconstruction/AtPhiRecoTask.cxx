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

AtPhiRecoTask::~AtPhiRecoTask() {}

void AtPhiRecoTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}
void AtPhiRecoTask::SetThreshold(Double_t threshold)
{
   fThreshold = threshold;
}
void AtPhiRecoTask::SetPhiRecoMode(Int_t value)
{
   fPhiRecoMode = value;
}

InitStatus AtPhiRecoTask::Init()
{

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fEventHArray = (TClonesArray *)ioMan->GetObject("AtEventH");
   if (fEventHArray == 0) {
      LOG(error) << "Cannot find AtEvent array!";
      return kERROR;
   }

   if (fPhiRecoMode == 0) {
      LOG(info) << "Using AtPhiRecoSimple!";

      fPhiReco = new AtPhiRecoSimple();
   } else if (fPhiRecoMode == 1) {
      LOG(info) << "Using AtPhiRecoTriple!";

      fPhiReco = new AtPhiRecoTriple();
   }

   // fPSA -> SetThreshold((Int_t)fThreshold);

   ioMan->Register("AtProtoEvent", "AtTPC", fPEventArray, fIsPersistence);

   return kSUCCESS;
}

void AtPhiRecoTask::SetParContainers()
{

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      LOG(fatal) << "AtDigiPar not found!!";
}

void AtPhiRecoTask::Exec(Option_t *opt)
{
   fPEventArray->Delete();

   if (fEventHArray->GetEntriesFast() == 0)
      return;

   AtEvent *event = (AtEvent *)fEventHArray->At(0);

   // std::cout << "  Event Number :  " << Event -> GetEventID() << std::endl;
   AtProtoEvent *protoevent = (AtProtoEvent *)new ((*fPEventArray)[0]) AtProtoEvent();
   protoevent->SetEventID(event->GetEventID());
   fPhiReco->PhiAnalyze(event, protoevent);
}
