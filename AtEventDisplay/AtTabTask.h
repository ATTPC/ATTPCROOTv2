#ifndef ATTABTASK_H
#define ATTABTASK_H

#include "AtTabBase.h"

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <memory>
#include <vector>        // for vector
class TClass;

class AtTabTask : public FairTask {
protected:
   std::vector<std::unique_ptr<AtTabBase> > fTabs;

   TTree *fTree;
   TString fTreeName;

public:
   AtTabTask();

   ~AtTabTask();

   InitStatus Init() override;
   void Exec(Option_t *option) override;
   void Reset();

   void MakeTab();
   void DrawPad(Int_t PadNum);

   void AddTab(std::unique_ptr<AtTabBase> tab) { fTabs.push_back(std::move(tab)); }

   ClassDefOverride(AtTabTask, 1);
};

#endif
