#include "AtMCFitterTask.h"

#include "AtMCFitter.h"
#include "AtPatternEvent.h"

#include <TObject.h>
InitStatus AtMCFitterTask::Init()
{
   fFitter->Init();

   FairRootManager *ioman = FairRootManager::Instance();
   ioman->Register("SimEvent", "cbmsim", &fFitter->GetEventArray(), false);
   ioman->Register("SimRawEvent", "cbmsim", &fFitter->GetRawEventArray(), false);
   fPatternArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtPatternEvent"));

   return kSUCCESS;
}

void AtMCFitterTask::Exec(Option_t *)
{
   auto patEvent = dynamic_cast<AtPatternEvent *>(fPatternArray->At(0));
   fFitter->Exec(*patEvent);
}
