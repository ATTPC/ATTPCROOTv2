#include "ATVMEUnpackTask.hh"


#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

ClassImp(ATVMEUnpackTask);

ATVMEUnpackTask::ATVMEUnpackTask()
{
  fLogger = FairLogger::GetLogger();
  fVMEDecoder = NULL;
  fDataNum = 0;
  fIsPersistence = kFALSE;
  fEventID=0;
  fICChannel=-1;
  fMeshChannel=-1;
  fTriggerChannel=-1;
  fVMERawEventArray = new TClonesArray("VMERawEvent");
}

ATVMEUnpackTask::~ATVMEUnpackTask()
{
}

void ATVMEUnpackTask::SetPersistence(Bool_t value)                           { fIsPersistence = value; }
void ATVMEUnpackTask::AddData(TString filename)                              { fDataList.push_back(filename); }
void ATVMEUnpackTask::SetData(Int_t value)                                   { fDataNum = value; }
void ATVMEUnpackTask::SetICChannel(Int_t value)				     { fICChannel = value; }
void ATVMEUnpackTask::SetMeshChannel(Int_t value)			     { fMeshChannel = value; }
void ATVMEUnpackTask::SetTriggerChannel(Int_t value)			     { fTriggerChannel = value; }

InitStatus
ATVMEUnpackTask::Init()
{
  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");

    return kERROR;
  }

   ioMan -> Register("VMERawEvent", "ATTPC", fVMERawEventArray, fIsPersistence);

   fVMEDecoder = new VMECore();
  
  for (Int_t iFile = 0; iFile < fDataList.size(); iFile++)
  fVMEDecoder -> AddData(fDataList.at(iFile));
  fVMEDecoder -> SetData(fDataNum);
  fVMEDecoder -> SetICChannel(fICChannel);
  fVMEDecoder -> SetMeshChannel(fMeshChannel);
  fVMEDecoder -> SetTriggerChannel(fTriggerChannel);

  /* if(fDebug)
       fDecoder->SetDebugMode(fDebug);*/

  return kSUCCESS;
}

void
ATVMEUnpackTask::SetParContainers()
{
  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find ATDigiPar!");
}

void
ATVMEUnpackTask::Exec(Option_t *opt)
{
   fVMERawEventArray -> Delete();

  
  
   VMERawEvent *rawVMEEvent =  fVMEDecoder -> GetRawVMEEvent(fEventID++);
   
    if(rawVMEEvent !=NULL)
       new ((*fVMERawEventArray)[0]) VMERawEvent(rawVMEEvent);
   else std::exit(1);
	
}

