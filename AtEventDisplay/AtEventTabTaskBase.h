#ifndef ATEVENTTABTASKBASE_H
#define ATEVENTTABTASKBASE_H

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

class AtEventTabTaskBase : public FairTask {
protected:
   std::unique_ptr<AtEventTab> fEventTab;

   Int_t fTaskNumber;

public:
   AtEventTabTaskBase(std::unique_ptr<AtEventTab> eventTab);

   ~AtEventTabTaskBase();

   virtual void Exec(Option_t *option);
   virtual void Reset();

   virtual void MakeTab();
   virtual void DrawPad(Int_t PadNum);
   virtual void SetTaskNumber(Int_t taskNum) { fTaskNumber = taskNum; }

   ClassDef(AtEventTabTaskBase, 1);
};

#endif
