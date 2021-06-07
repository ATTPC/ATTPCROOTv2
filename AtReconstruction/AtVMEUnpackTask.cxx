#include "AtVMEUnpackTask.h"

#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

ClassImp(AtVMEUnpackTask);

AtVMEUnpackTask::AtVMEUnpackTask()
{
   fLogger = FairLogger::GetLogger();
   fVMEDecoder = NULL;
   fDataNum = 0;
   fIsPersistence = kFALSE;
   fEventID = 0;
   fICChannel = -1;
   fMeshChannel = -1;
   fTriggerChannel = -1;
   fVMERawEventArray = new TClonesArray("VMERawEvent");
}

AtVMEUnpackTask::~AtVMEUnpackTask() {}

void AtVMEUnpackTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}
void AtVMEUnpackTask::AddData(TString filename)
{
   fDataList.push_back(filename);
}
void AtVMEUnpackTask::SetData(Int_t value)
{
   fDataNum = value;
}
void AtVMEUnpackTask::SetICChannel(Int_t value)
{
   fICChannel = value;
}
void AtVMEUnpackTask::SetMeshChannel(Int_t value)
{
   fMeshChannel = value;
}
void AtVMEUnpackTask::SetTriggerChannel(Int_t value)
{
   fTriggerChannel = value;
}

InitStatus AtVMEUnpackTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(error) << "Cannot find RootManager!";

      return kERROR;
   }

   ioMan->Register("VMERawEvent", "AtTPC", fVMERawEventArray, fIsPersistence);

   fVMEDecoder = new VMECore();

   for (Int_t iFile = 0; iFile < fDataList.size(); iFile++)
      fVMEDecoder->AddData(fDataList.at(iFile));
   fVMEDecoder->SetData(fDataNum);
   fVMEDecoder->SetICChannel(fICChannel);
   fVMEDecoder->SetMeshChannel(fMeshChannel);
   fVMEDecoder->SetTriggerChannel(fTriggerChannel);

   /* if(fDebug)
        fDecoder->SetDebugMode(fDebug);*/

   return kSUCCESS;
}

void AtVMEUnpackTask::SetParContainers()
{
   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      LOG(fatal) << "Cannot find AtDigiPar!";
}

void AtVMEUnpackTask::Exec(Option_t *opt)
{
   fVMERawEventArray->Delete();

   VMERawEvent *rawVMEEvent = fVMEDecoder->GetRawVMEEvent(fEventID++);

   if (rawVMEEvent != NULL)
      new ((*fVMERawEventArray)[0]) VMERawEvent(rawVMEEvent);
   else
      std::exit(1);
}
