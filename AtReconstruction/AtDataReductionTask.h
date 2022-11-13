#ifndef ATDATAREDUCTIONTSAK_H
#define ATDATAREDUCTIONTSAK_H
/*
 * Task to reduce the data. It takes a function pointer of the type:
 * bool func(AtRawEvent*)
 * If the function returns true, it keeps the event. Otherwise it skips writing
 * it to disk
 *
 *
 *
 */

#include <FairTask.h>

#include <Rtypes.h>
#include <TString.h>

#include <functional> // for function
#include <utility>    // for move

class AtRawEvent;
class TClonesArray;

class AtDataReductionTask : public FairTask {
   using ReductionFunction = std::function<bool(AtRawEvent *)>;

private:
   ReductionFunction fReductionFunction;

   TClonesArray *fInputEventArray{}; // AtRawEvent
   TString fInputBranchName;         // Name if AtRawEvent branch
   AtRawEvent *fRawEvent{};

public:
   AtDataReductionTask();
   ~AtDataReductionTask();

   void SetReductionFunction(ReductionFunction func) { fReductionFunction = std::move(func); }
   void SetInputBranch(TString inputBranch) { fInputBranchName = inputBranch; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
};

#endif //#ifndef ATDATAREDUCTIONTSAK_H
