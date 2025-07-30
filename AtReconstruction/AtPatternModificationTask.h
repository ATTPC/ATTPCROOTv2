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
   TString fPatternEventBranchName;
   TString fRawEventBranchName;
   TString fEventBranchName;

   std::vector<std::unique_ptr<AtPatternModification>> fPatternModifications;
   TClonesArray *fPatternEventArray;
   TClonesArray *fRawEventArray;
   TClonesArray *fEventArray;

   ULong_t fEventCnt{0};

public:
   AtPatternModificationTask();
   ~AtPatternModificationTask() = default;

   void SetPatternEventBranch(TString branchName);
   void SetRawEventBranch(TString branchName);
   void SetEventBranch(TString branchName);

   void AddPatternModification(std::unique_ptr<AtPatternModification> patternModification)
   {
      fPatternModifications.push_back(std::move(patternModification));
   }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

private:
   ClassDefOverride(AtPatternModificationTask, 1);
};

#endif
