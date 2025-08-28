#ifndef ATPATTERNMODIFICATIONTASK
#define ATPATTERNMODIFICATIONTASK

#include "AtPatternModification.h"

#include <FairTask.h>

#include <Rtypes.h>
#include <TClonesArray.h>
#include <TString.h>

#include <memory>
#include <utility>
#include <vector>

class AtPatternModificationTask : public FairTask {
private:
   TString fInputBranchName;
   TString fOutputBranchName;
   TString fRawEventBranchName;
   TString fEventBranchName;

   std::vector<std::unique_ptr<AtPatternModification>> fPatternModifications;
   TClonesArray *fPatternEventArray;
   TClonesArray *fRawEventArray;
   TClonesArray *fEventArray;
   TClonesArray fPatternEventModifiedArray;

   Bool_t kIsPersistence{kFALSE};

   ULong_t fEventCnt{0};

public:
   AtPatternModificationTask(std::vector<std::unique_ptr<AtPatternModification>> patternModifications);
   ~AtPatternModificationTask() = default;

   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   void SetRawEventBranch(TString branchName);
   void SetEventBranch(TString branchName);
   void SetPersistence(Bool_t value = kTRUE);

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

private:
   ClassDefOverride(AtPatternModificationTask, 1);
};

#endif
