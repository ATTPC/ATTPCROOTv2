#include "AtMCFitterTask.h"

#include "AtMCFitter.h"
#include "AtPatternEvent.h"

#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h> // for TClonesArray
#include <TObject.h>

InitStatus AtMCFitterTask::Init()
{
   LOG(debug) << "Initialing fitter";
   fFitter->Init();

   FairRootManager *ioman = FairRootManager::Instance();
   ioman->Register("SimEvent", "cbmsim", &fFitter->GetEventArray(), false);
   ioman->Register("SimRawEvent", "cbmsim", &fFitter->GetRawEventArray(), false);
   ioman->Register("AtMCResult", "cbmsim", &fResultArray, false);

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
   fFitter->Exec(*patEvent);
   fFitter->FillResultArray(fResultArray);
}
