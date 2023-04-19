#ifndef ATTABMAIN_H
#define ATTABMAIN_H
#include "AtDataObserver.h"         // for Observer
#include "AtTabBase.h"              // for AtTabBase
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
 * @brief Main tab in viewer for 3D and pad selection.
 */
class AtTabMain : public AtTabBase, public DataHandling::AtObserver {
protected:
   using TEvePointSetPtr = std::unique_ptr<TEvePointSet>;
   using TEveEventManagerPtr = std::unique_ptr<TEveEventManager>;

   TEveEventManagerPtr fEveEvent{std::make_unique<TEveEventManager>("AtEvent")};
   TEvePointSetPtr fHitSet{std::make_unique<TEvePointSet>("Hits")}; //< AtEvent Hit Set

   TEveEventManagerPtr fEvePatternEvent{std::make_unique<TEveEventManager>("AtPatternEvent")};
   TEvePointSetPtr fNoiseHitSet{std::make_unique<TEvePointSet>("Noise")}; //< AtPatternEvent Noise Set
   std::vector<TEvePointSetPtr> fPatternHitSets;
   std::vector<TEveElement> fPatterns;

   Int_t fThreshold{0};    //< Min charge to draw hit
   Int_t fMaxHitMulti{10}; //< Max hits in a pad for hit to be drawn

   TAttMarker fHitAttr{kPink, kFullDotMedium, 1};

   TCanvas *fCvsPadPlane{nullptr};
   TH2Poly *fPadPlane{nullptr};

   TCanvas *fCvsPadWave{nullptr};
   TH1I *fPadWave{nullptr};
   DataHandling::AtPadNum *fPadNum;
   DataHandling::AtBranch *fEventBranch;
   DataHandling::AtBranch *fRawEventBranch;
   DataHandling::AtBranch *fPatternEventBranch;
   DataHandling::AtTreeEntry *fEntry;

public:
   AtTabMain();
   ~AtTabMain();
   void InitTab() override;

   void Exec() override{};
   void Update(DataHandling::AtSubject *sub) override;

   void DumpEvent(std::string file);

   void SetThreshold(Int_t val) { fThreshold = val; }
   void SetHitAttributes(TAttMarker attr) { fHitAttr = std::move(attr); }
   void SetMultiHit(Int_t hitMax) { fMaxHitMulti = hitMax; }

   /**
    * This function is responsible for selecting the pad we are currently examining and passing
    * that onto AtViewerManager.
    */
   static void SelectPad();

protected:
   void MakeTab(TEveWindowSlot *slot) override;

   // Sets the default render state for TEveEventManagers
   virtual void UpdateRenderState();
   Color_t GetTrackColor(int i);
   void SetPointsFromHits(TEvePointSet &hitSet, const std::vector<std::unique_ptr<AtHit>> &hits);
   void SetPointsFromHits(TEvePointSet &hitSet, const std::vector<AtHit *> &hits);
   void SetPointsFromTrack(TEvePointSet &hitSet, const AtTrack &track);

private:
   // Functions to draw the initial canvases
   void DrawPadPlane();
   void DrawPadWave();

   bool DrawWave(Int_t PadNum);

   // Update hit sets
   void UpdatePadPlane();
   void UpdateEventElements();
   void UpdatePatternEventElements();

   void ExpandNumPatterns(int num);

   ClassDefOverride(AtTabMain, 1)
};

#endif
