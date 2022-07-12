#include "AtSampleConsensusTask.h"

#include "AtEvent.h" // for AtEvent
#include "AtPatternEvent.h"
#include "AtSampleConsensus.h"

#include <FairLogger.h>      // for LOG, Logger
#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h> // for TClonesArray
#include <TObject.h>      // for TObject

#include <memory>  // for allocator
#include <utility> // for move

ClassImp(AtSampleConsensusTask);

AtSampleConsensusTask::AtSampleConsensusTask(std::unique_ptr<SampleConsensus::AtSampleConsensus> method)
   : fInputBranchName("AtEventH"), fOutputBranchName("AtPatternEvent"), fPatternEventArray("AtPatternEvent", 1),
     fSampleConsensus(std::move(method)), kIsPersistence(kFALSE)
{
}

InitStatus AtSampleConsensusTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));
   if (fEventArray == nullptr) {

      LOG(error) << "Cannot find AtEvent array!";
      return kERROR;
   }

   ioMan->Register(fOutputBranchName, "AtTPC", &fPatternEventArray, kIsPersistence);

   return kSUCCESS;
}

void AtSampleConsensusTask::Exec(Option_t *opt)
{

   if (fEventArray->GetEntriesFast() == 0)
      return;

   fEvent = dynamic_cast<AtEvent *>(fEventArray->At(0));

   LOG(debug) << "Running Sample Consensus with " << fEvent->GetNumHits() << " hits.";

   fPatternEventArray.Delete();
   auto patternEvent = fSampleConsensus->Solve(fEvent);
   new (fPatternEventArray[0]) AtPatternEvent(patternEvent);
}
