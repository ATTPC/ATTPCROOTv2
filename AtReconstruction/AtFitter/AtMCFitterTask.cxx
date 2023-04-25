#include "AtMCFitterTask.h"

#include "AtMCFitter.h"
#include "AtPatternEvent.h"

#include <FairLogger.h>      // for LOG, Logger
#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h> // for TClonesArray
#include <TObject.h>

#include <utility>

AtMCFitterTask::AtMCFitterTask(std::shared_ptr<MCFitter::AtMCFitter> fitter)
   : fFitter(std::move(fitter)), fResultArray("MCFitter::AtMCResult"), fSimEventArray("AtEvent"),
     fSimRawEventArray("AtRawEvent")
{
}

InitStatus AtMCFitterTask::Init()
{
   LOG(debug) << "Initialing fitter";
   fFitter->Init();

   FairRootManager *ioman = FairRootManager::Instance();
   ioman->Register("SimEvent", "cbmsim", &fSimEventArray, fSaveEvent);
   ioman->Register("SimRawEvent", "cbmsim", &fSimRawEventArray, fSaveRawEvent);
   ioman->Register("AtMCResult", "cbmsim", &fResultArray, fSaveResult);

   fPatternArray = dynamic_cast<TClonesArray *>(ioman->GetObject(fPatternBranchName));
   if (fPatternArray == nullptr)
      LOG(fatal) << "Failed to load branch " << fPatternBranchName;

   LOG(debug) << "Done with sim init";
   return kSUCCESS;
}

void AtMCFitterTask::Exec(Option_t *)
{
   LOG(debug) << "Exec";
   auto patEvent = dynamic_cast<AtPatternEvent *>(fPatternArray->At(0));
   if (!patEvent->IsGood())
      return;

   fFitter->Exec(*patEvent);
   fResultArray.Delete();
   fSimEventArray.Delete();
   fSimRawEventArray.Delete();

   fFitter->FillResultArrays(fResultArray, fSimEventArray, fSimRawEventArray);
}
