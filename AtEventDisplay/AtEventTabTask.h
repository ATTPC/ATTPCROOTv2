#ifndef ATEVENTTABTASK_H
#define ATEVENTTABTASK_H

#include "AtEventTabTaskBase.h"
#include "AtEventTab.h"

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <memory>
#include <vector>        // for vector
class AtEventManagerNew; // lines 17-17
class AtRawEvent;        // lines 22-22
class TBuffer;
class TCanvas; // lines 30-30
class TClass;
class TClonesArray;    // lines 31-31
class TMemberInspector;

class AtEventTabTask : public AtEventTabTaskBase {
protected:
   AtEventManagerNew *fEventManager;

   TString fRawEventBranchName;
   TString fEventBranchName;

   TClonesArray *fRawEventArray{};
   TClonesArray *fEventArray{};

   AtRawEvent *fRawEvent;
   AtEvent *fEvent;

public:
   AtEventTabTask(std::unique_ptr<AtEventTab> eventTab);

   ~AtEventTabTask();

   InitStatus Init() override;
   void Exec(Option_t *option) override;

   void SetRawEventBranch(TString branchName);
   void SetEventBranch(TString branchName);

   ClassDefOverride(AtEventTabTask, 1);
};

#endif
