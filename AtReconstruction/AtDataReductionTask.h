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

#include <TString.h>
#include <FairTask.h>
#include <Rtypes.h>
#include <type_traits>

class AtRawEvent;
class TClonesArray;

class AtDataReductionTask : public FairTask {
   // using funcType = bool (*)(AtRawEvent*);
   using funcType = std::add_pointer<bool(AtRawEvent *)>::type;

private:
   funcType reduceFunc;

   TClonesArray *fInputEventArray{}; // AtRawEvent
   TString fInputBranchName;         // Name if AtRawEvent branch
   AtRawEvent *fRawEvent{};

public:
   AtDataReductionTask();
   ~AtDataReductionTask();

   void SetReductionFunction(funcType func) { reduceFunc = func; }
   void SetInputBranch(TString inputBranch) { fInputBranchName = inputBranch; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
};

#endif //#ifndef ATDATAREDUCTIONTSAK_H
