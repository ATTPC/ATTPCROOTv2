#ifndef ATMCFITTERTASK_H
#define ATMCFITTERTASK_H

#include <FairTask.h>

#include <TClonesArray.h>
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
