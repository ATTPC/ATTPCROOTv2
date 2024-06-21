/* Task for correcting SpaceCharge effects by
 * applying some SpaceChargeModel and
 * modifying the position values of hits in AtEvent
 *
 */
#ifndef ATDATACLEANINGTASK_H
#define ATDATACLEANINGTASK_H

#include "AtDataCleaner.h" // IWYU pragma: keep

#include <FairTask.h>

#include <Rtypes.h>
#include <TClonesArray.h>

#include <memory>
#include <string>

class TBuffer;
class TClass;
class TMemberInspector;

using DataCleaner = std::unique_ptr<AtTools::DataCleaning::AtDataCleaner>;

class AtDataCleaningTask : public FairTask {

private:
   std::string fInputBranchName = "AtEventH";
   std::string fOuputBranchName = "AtEventCleaned";
   Bool_t fIsPersistent = true;

   TClonesArray *fInputEventArray;
   TClonesArray fOutputEventArray;
   DataCleaner fCleaner;

public:
   AtDataCleaningTask(DataCleaner &&cleaner);
   virtual ~AtDataCleaningTask() = default;

   void SetInputBranch(std::string branchName) { fInputBranchName = branchName; }
   void SetOutputBranch(std::string branchName) { fOuputBranchName = branchName; }
   void SetPersistence(Bool_t value) { fIsPersistent = value; }

   virtual InitStatus Init() override;
   virtual void SetParContainers() override;
   virtual void Exec(Option_t *opt) override;
   // virtual void FinishEvent() override;

   ClassDefOverride(AtDataCleaningTask, 1);
};

#endif //_ATSPACECHARGETASK_H_
