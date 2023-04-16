#ifndef ATMCFITTERTASK_H
#define ATMCFITTERTASK_H

#include <FairTask.h>

#include <Rtypes.h> // for Option_t

#include <memory> // for shared_ptr

class TClonesArray;
namespace MCFitter {
class AtMCFitter;
}

class AtMCFitterTask : public FairTask {

   std::shared_ptr<MCFitter::AtMCFitter> fFitter; //!
   TClonesArray *fPatternArray{nullptr};

public:
   AtMCFitterTask(std::shared_ptr<MCFitter::AtMCFitter> fitter) : fFitter(fitter){};

   InitStatus Init() override;
   void Exec(Option_t *option = "") override;
   void Finish() override{};
};

#endif // ATMCFITTERTASK_H
