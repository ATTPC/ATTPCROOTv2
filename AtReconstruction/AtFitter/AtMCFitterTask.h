#ifndef ATMCFITTERTASK_H
#define ATMCFITTERTASK_H

#include <FairTask.h>

#include <Rtypes.h> // for Option_t
#include <TClonesArray.h>

#include <memory> // for shared_ptr

namespace MCFitter {
class AtMCFitter;
}

class AtMCFitterTask : public FairTask {

   std::shared_ptr<MCFitter::AtMCFitter> fFitter; //!
   TString fPatternBranchName{"AtPatternEvent"};
   TClonesArray *fPatternArray{nullptr};

   TClonesArray fResultArray; //< Output of task

public:
   AtMCFitterTask(std::shared_ptr<MCFitter::AtMCFitter> fitter)
      : fFitter(fitter), fResultArray("MCFitter::AtMCResult"){};

   InitStatus Init() override;
   void Exec(Option_t *option = "") override;
   void Finish() override{};

   void SetPatternBranchName(TString name) { fPatternBranchName = name; }
};

#endif // ATMCFITTERTASK_H
