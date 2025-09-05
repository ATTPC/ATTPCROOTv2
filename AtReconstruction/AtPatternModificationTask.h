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

/**
 * This task takes the AtPatternEvent coming from the pattern recognition task and makes a copy of it. Then, it takes
 * this new AtPatternEvent and it applies a list of AtPatternModifications to it. The resulting final modified
 * AtPatternEvent is stored in a new TBranch. It can also be saved in the original AtPatternEvent TBranch if you set the
 * output name to it and you set kIsPersistence to false in the pattern recognition task.
 */
class AtPatternModificationTask : public FairTask {
private:
   TString fInputBranchName{"AtPatternEvent"};
   TString fOutputBranchName{"AtPatternEventModified"};
   TString fRawEventBranchName{"AtRawEvent"};
   TString fEventBranchName{"AtEvent"};

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
