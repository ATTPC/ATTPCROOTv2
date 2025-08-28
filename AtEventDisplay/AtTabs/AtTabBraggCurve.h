#ifndef ATTABBRAGGCURVE_H
#define ATTABBRAGGCURVE_H

#include "AtTabMain.h" // for AtTabMain::TEvePointSetPtr, AtTa...
#include "AtTrack.h"
#include "AtViewerManagerSubject.h" // for AtBranch

#include <Rtypes.h>           // for THashConsistencyHolder, ClassDef...
#include <TEveEventManager.h> // for TEveEventManager
#include <TEvePointSet.h>     // for TEvePointSet
#include <TH1F.h>

#include <array>        // for array
#include <memory>       // for make_unique
class TBuffer;          // lines 21-21
class TClass;           // lines 23-23
class TMemberInspector; // lines 27-27
namespace DataHandling {
class AtSubject;
}

/**
 * @brief Tab for visualizing the Bragg curves of an AtTrack
 */
class AtTabBraggCurve : public AtTabMain {
protected:
   TCanvas *fCvsELossVRange{nullptr};
   TH1F *fHistELossVRange{nullptr};

   int fTrackIdx{-1};

public:
   AtTabBraggCurve();
   virtual ~AtTabBraggCurve();

   virtual void Update(DataHandling::AtSubject *sub) override;

protected:
   virtual void MakeTab(TEveWindowSlot *slot) override;

   void DrawHistELossVRange();
   void DrawHistELossVRange(AtTrack::BraggCurve braggCurve);

   virtual void UpdatePatternEventElements() override;

private:
   ClassDefOverride(AtTabBraggCurve, 1);
};
#endif
