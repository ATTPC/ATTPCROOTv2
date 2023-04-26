#ifndef ATMCFITTERTASK_H
#define ATMCFITTERTASK_H

#include <FairTask.h>

#include <Rtypes.h> // for Option_t
#include <TClonesArray.h>
#include <TString.h> // for TString

#include <memory> // for shared_ptr

namespace MCFitter {
class AtMCFitter;
}

class AtMCFitterTask : public FairTask {

   std::shared_ptr<MCFitter::AtMCFitter> fFitter; //!
   TString fPatternBranchName{"AtPatternEvent"};
   TClonesArray *fPatternArray{nullptr};

   TClonesArray fResultArray; //< Output of task
   TClonesArray fSimEventArray;
   TClonesArray fSimRawEventArray;

   Bool_t fSaveResult{true};
   Bool_t fSaveEvent{false};
   Bool_t fSaveRawEvent{false};

public:
   AtMCFitterTask(std::shared_ptr<MCFitter::AtMCFitter> fitter);

   InitStatus Init() override;
   void Exec(Option_t *option = "") override;
   void Finish() override{};

   void SetPatternBranchName(TString name) { fPatternBranchName = name; }
   void SetSaveResult(bool val) { fSaveResult = val; }
   void SetSaveEvent(bool val) { fSaveEvent = val; }
   void SetSaveRawEvent(bool val) { fSaveRawEvent = val; }
};

#endif // ATMCFITTERTASK_H
