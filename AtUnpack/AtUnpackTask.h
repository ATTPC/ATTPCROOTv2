/* Task for unpacking data into a AtRawEvent
 * the logic for the unpacker is passed to the
 * task as a pointer to AtUnpacker
 *
 */
#ifndef _ATUNPACKTASK_H_
#define _ATUNPACKTASK_H_

#include <Rtypes.h>
#include <RtypesCore.h>
#include <memory>
#include <string>

#include <FairTask.h>
#include "AtUnpacker.h"
#include <TClonesArray.h>

class AtRawEvent;
class AtMap;
class TBuffer;
class TClass;
class TMemberInspector;

using mapPtr = std::shared_ptr<AtMap>;
using unpackerPtr = std::unique_ptr<AtUnpacker>;

class AtUnpackTask : public FairTask {

private:
   std::string fInputFileName;
   std::string fOuputBranchName = "AtRawEvent";
   Bool_t fIsPersistent = true;

   TClonesArray fOutputEventArray;
   AtRawEvent *fRawEvent;
   unpackerPtr fUnpacker;

public:
   AtUnpackTask(unpackerPtr unpacker);
   ~AtUnpackTask() = default;

   void SetInputFileName(std::string filename) { fInputFileName = filename; }
   void SetOuputBranchName(std::string branchName) { fOuputBranchName = branchName; }
   void SetPersistence(Bool_t value) { fIsPersistent = value; }
   // void SetMap(mapPtr map) { fAtMapPtr = map; }
   // void SetInitialEvent(std::size_t inievent) { fIniEventID = inievent; }

   Long64_t GetNumEvents() { return fUnpacker->GetNumEvents(); }

   virtual InitStatus Init() override;
   virtual void SetParContainers() override;
   virtual void Exec(Option_t *opt) override;
   virtual void FinishEvent() override;

   ClassDefOverride(AtUnpackTask, 1);
};

#endif //_ATUNPACKERTASK_H_
