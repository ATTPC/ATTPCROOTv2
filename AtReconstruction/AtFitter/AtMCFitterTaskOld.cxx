#include "AtMCFitterTaskOld.h"

#include "AtMCFitterOld.h"
#include "AtPatternEvent.h"

#include <FairLogger.h>      // for LOG, Logger
#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h> // for TClonesArray
#include <TObject.h>

#include <utility>

AtMCFitterTaskOld::AtMCFitterTaskOld(std::shared_ptr<MCFitter::AtMCFitterOld> fitter)
   : fFitter(std::move(fitter)), fResultArray("MCFitter::AtMCResultOld"), fSimEventArray("AtEvent"),
     fSimRawEventArray("AtRawEvent")
{
}

InitStatus AtMCFitterTaskOld::Init()
{
   LOG(debug) << "Initialing fitter";
   fFitter->Init();

   FairRootManager *ioman = FairRootManager::Instance();
   ioman->Register("SimEvent", "cbmsim", &fSimEventArray, fSaveEvent);
   ioman->Register("SimRawEvent", "cbmsim", &fSimRawEventArray, fSaveRawEvent);
   ioman->Register("AtMCResultOld", "cbmsim", &fResultArray, fSaveResult);

   fPatternArray = dynamic_cast<TClonesArray *>(ioman->GetObject(fPatternBranchName));
   if (fPatternArray == nullptr)
      LOG(fatal) << "Failed to load branch " << fPatternBranchName;

   LOG(debug) << "Done with sim init";
   return kSUCCESS;
}

void AtMCFitterTaskOld::Exec(Option_t *)
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
