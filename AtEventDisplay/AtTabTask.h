#ifndef ATTABTASK_H
#define ATTABTASK_H

#include "AtTabBase.h"

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <algorithm> // for max
#include <memory>
#include <utility> // for move
#include <vector>  // for vector
class TBuffer;
class TMemberInspector;
class TTree;
class TClass;
class SubjectBase;

class AtTabTask : public FairTask {
protected:
   std::vector<std::unique_ptr<AtTabBase>> fTabs;

   TTree *fTree{nullptr};
   TString fTreeName;

public:
   AtTabTask() {}

   virtual ~AtTabTask() = default;

   InitStatus Init() override;
   void Exec(Option_t *option) override;
   void Reset();

   void MakeTab();
   void DrawPad(Int_t PadNum);

   void AddTab(std::unique_ptr<AtTabBase> tab) { fTabs.push_back(std::move(tab)); }

   void AddDataSourceToTabs(SubjectBase *subject);

   ClassDefOverride(AtTabTask, 1);
};

#endif
