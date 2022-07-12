#ifndef AtSAMPLECONSENSUSTASK_H
#define AtSAMPLECONSENSUSTASK_H

#include "AtSampleConsensus.h"

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h> // for Int_t, Bool_t, Double_t, THashConsistencyHolder
#include <TClonesArray.h>
#include <TString.h> // for TString

#include <memory> // for unique_ptr

class AtEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class AtSampleConsensusTask : public FairTask {
private:
   TString fInputBranchName;
   TString fOutputBranchName;

   TClonesArray *fEventArray{};
   TClonesArray fPatternEventArray;

   AtEvent *fEvent{};

   std::unique_ptr<SampleConsensus::AtSampleConsensus> fSampleConsensus;
   Bool_t kIsPersistence;

public:
   AtSampleConsensusTask(std::unique_ptr<SampleConsensus::AtSampleConsensus> method);

   void SetInputBranch(TString branchName) { fInputBranchName = branchName; }
   void SetOutputBranch(TString branchName) { fOutputBranchName = branchName; }
   void SetPersistence(Bool_t value = kTRUE) { kIsPersistence = value; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

   ClassDefOverride(AtSampleConsensusTask, 1);
};

#endif // AtSAMPLECONSENSUSTASK_H
