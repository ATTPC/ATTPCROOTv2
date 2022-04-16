#ifndef ATFILTERTASK_H
#define ATFILTERTASK_H

#include <TString.h>
// FairRoot classes
#include <FairTask.h>

#include <Rtypes.h>

// ATTPCROOT classes;
class AtFilter;
// ROOT classes
class TClonesArray;

class AtFilterTask : public FairTask {

private:
   TClonesArray *fInputEventArray{}; // AtRawEvent
   TClonesArray *fOutputEventArray;  // AtRawEvent

   AtFilter *fFilter;
   Bool_t fIsPersistent;
   Bool_t fFilterAux;

   TString fInputBranchName;
   TString fOutputBranchName;

public:
   AtFilterTask(AtFilter *filter);
   ~AtFilterTask();

   void SetPersistence(Bool_t value);
   void SetFilterAux(Bool_t value);
   void SetInputBranch(TString name) { fInputBranchName = name; }
   void SetOutputBranch(TString name) { fOutputBranchName = name; }
   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
};
#endif //#ifndef ATFILTERTASK_H
