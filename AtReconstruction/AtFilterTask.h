#ifndef ATFILTERTASK_H
#define ATFILTERTASK_H

#include <FairTask.h>

#include <Rtypes.h>
#include <TString.h>

class AtFilter;
class TClonesArray;

class AtFilterTask : public FairTask {

private:
   TClonesArray *fInputEventArray{nullptr}; // AtRawEvent
   TClonesArray *fOutputEventArray;         // AtRawEvent

   AtFilter *fFilter;
   Bool_t fIsPersistent{false};
   Bool_t fFilterAux{false};
   Bool_t fFilterFPN{false};
   Bool_t fFilterPads{true};

   TString fInputBranchName{"AtRawEvent"};
   TString fOutputBranchName{"AtRawEventFiltered"};

public:
   AtFilterTask(AtFilter *filter, const char *name = "AtFilterTask");
   ~AtFilterTask() = default;

   void SetPersistence(Bool_t value) { fIsPersistent = value; }
   void SetFilterPads(Bool_t value) { fFilterPads = value; }
   void SetFilterAux(Bool_t value) { fFilterAux = value; }
   void SetFilterFPN(Bool_t value) { fFilterFPN = value; }
   void SetInputBranch(TString name) { fInputBranchName = name; }
   void SetOutputBranch(TString name) { fOutputBranchName = name; }
   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
};
#endif //#ifndef ATFILTERTASK_H
