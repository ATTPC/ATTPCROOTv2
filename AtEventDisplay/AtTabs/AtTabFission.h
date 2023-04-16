#ifndef ATTABFISSION_H
#define ATTABFISSION_H
#include "AtDataObserver.h" // for Observer
#include "AtTabBase.h"      // for AtTabBase
#include "AtTabMain.h"
#include "AtViewerManagerSubject.h" // for AtPadNum

#include <Rtypes.h>           // for Int_t, THashConsistencyHolder
#include <TAttMarker.h>       // for TAttMarker, kFullDotMedium
#include <TEveElement.h>      // for TEveElement
#include <TEveEventManager.h> // for TEveEventManager
#include <TEvePointSet.h>     // for TEvePointSet

#include <memory>  // for make_unique, unique_ptr
#include <string>  // for string
#include <utility> // for move
#include <vector>  // for vector

class AtHit;
class AtTrack;
class TBuffer; // lines 19-19
class TCanvas; // lines 24-24
class TClass;  // lines 20-20
class TEveWindowSlot;
class TH1I;             // lines 28-28
class TH2Poly;          // lines 27-27
class TMemberInspector; // lines 21-21
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
   void Exec() override;

protected:
   void UpdateRenderState() override;

private:
   void UpdateFissionElements();

   ClassDefOverride(AtTabFission, 1);
};
#endif
