#ifndef ATTABFISSION_H
#define ATTABFISSION_H
#include "AtTabMain.h"              // for AtTabMain::TEvePointSetPtr, AtTa...
#include "AtViewerManagerSubject.h" // for AtBranch

#include <Rtypes.h>           // for THashConsistencyHolder, ClassDef...
#include <TEveEventManager.h> // for TEveEventManager
#include <TEvePointSet.h>     // for TEvePointSet

#include <array>        // for array
#include <memory>       // for make_unique
class TBuffer;          // lines 21-21
class TClass;           // lines 23-23
class TMemberInspector; // lines 27-27
namespace DataHandling {
class AtSubject;
}

/**
 * @brief Tab for hangling fission events in viewer
 */
class AtTabFission : public AtTabMain {
protected:
   TEveEventManagerPtr fEveFissionEvent{std::make_unique<TEveEventManager>("AtFissionEvent")};

   TEvePointSetPtr fUncorrHitSet{std::make_unique<TEvePointSet>("Uncorrected Hits")}; //< AtEvent Hit Set
   std::array<TEvePointSetPtr, 3> fCorrHitSet{};

   DataHandling::AtBranch fFissionEventBranch{};

public:
   AtTabFission();
   ~AtTabFission() = default;

   void InitTab() override;
   void Update(DataHandling::AtSubject *sub) override;

   DataHandling::AtBranch &GetFissionBranch() { return fFissionEventBranch; }

protected:
   void UpdateRenderState() override;

private:
   void UpdateFissionElements();

   ClassDefOverride(AtTabFission, 1);
};
#endif
