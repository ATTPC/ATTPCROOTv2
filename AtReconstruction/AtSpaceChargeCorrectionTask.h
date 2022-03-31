/* Task for correcting SpaceCharge effects by
 * applying some SpaceChargeModel and
 * modifying the position values of hits in AtEvent
 *
 */
#ifndef _ATSPACECHARGECORRECTIONTASK_H_
#define _ATSPACECHARGECORRECTIONTASK_H_

#include "FairTask.h"
#include "TClonesArray.h"
#include "AtSpaceChargeModel.h"

#include <memory>
#include <string>

class AtEvent;
// class AtSpaceChargeModel;

using SCModelPtr = std::unique_ptr<AtSpaceChargeModel>;

class AtSpaceChargeCorrectionTask : public FairTask {

private:
   std::string fInputBranchName = "AtEventH";
   std::string fOuputBranchName = "AtEventCorrected";
   Bool_t fIsPersistent = true;

   TClonesArray *fInputEventArray;
   TClonesArray fOutputEventArray;
   SCModelPtr fSCModel;

public:
   AtSpaceChargeCorrectionTask(SCModelPtr &&model);
   ~AtSpaceChargeCorrectionTask() = default;

   void SetInputBranchName(std::string branchName) { fInputBranchName = branchName; }
   void SetOuputBranchName(std::string branchName) { fOuputBranchName = branchName; }
   void SetPersistence(Bool_t value) { fIsPersistent = value; }

   virtual InitStatus Init() override;
   virtual void SetParContainers() override;
   virtual void Exec(Option_t *opt) override;
   // virtual void FinishEvent() override;

   ClassDefOverride(AtSpaceChargeCorrectionTask, 1);
};

#endif //_ATSPACECHARGETASK_H_
