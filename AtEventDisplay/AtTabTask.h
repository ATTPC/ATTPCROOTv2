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
namespace DataHandling {
class Subject;
}

/**
 * Class used by AtEventManagerNew to manage tabs and keeping them upto date as we move through events.
 */
class AtTabTask : public FairTask {
protected:
   std::vector<std::unique_ptr<AtTabBase>> fTabs;

public:
   void AddTab(std::unique_ptr<AtTabBase> tab);
   void AddDataSourceToTabs(DataHandling::Subject *subject);

   InitStatus Init() override;
   void Exec(Option_t *option) override;
   void DrawTabPads(Int_t PadNum);

private:
   void Reset();
   ClassDefOverride(AtTabTask, 1);
};

#endif
