#ifndef ATEVENTTABTASK_H
#define ATEVENTTABTASK_H

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

class AtEventTabTask : public FairTask {
protected:
   TString fRawEventBranchName;
   TString fEventBranchName;

   TClonesArray *fRawEventArray{};
   TClonesArray *fEventArray{};

   AtEventManagerNew *fEventManager;
   AtRawEvent *fRawEvent;
   AtEvent *fEvent;

   std::unique_ptr<AtEventTab> fEventTab;

   Int_t fTaskNumber;

public:
   AtEventTabTask(std::unique_ptr<AtEventTab> eventTab);

   ~AtEventTabTask();

   InitStatus Init();
   void Exec(Option_t *option);
   void Reset();

   void SetRawEventBranch(TString branchName);
   void SetEventBranch(TString branchName);
   void MakeTab();
   void DrawPad(Int_t PadNum);
   void SetTaskNumber(Int_t taskNum) { fTaskNumber = taskNum; }

private:
   ClassDef(AtEventTabTask, 2);
};

#endif
