#ifndef ATTABBRAGGCURVE_H
#define ATTABBRAGGCURVE_H

#include "AtFittedTrack.h"
#include "AtTabMain.h" // for AtTabMain::TEvePointSetPtr, AtTa...
#include "AtTrack.h"
#include "AtViewerManagerSubject.h" // for AtBranch

#include <Rtypes.h>           // for THashConsistencyHolder, ClassDef...
#include <TEveEventManager.h> // for TEveEventManager
#include <TEvePointSet.h>     // for TEvePointSet
#include <TH1F.h>
#include <TGraph.h>

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
   TGraph *fFittedELossGraph{nullptr};

   int fTrackIdx{-1};
   DataHandling::AtBranch *fTrackingEventBranch;
   DataHandling::AtBranch *fFitMetadataBranch;

public:
   AtTabBraggCurve();
   virtual ~AtTabBraggCurve();
   void InitTab() override;

   virtual void Update(DataHandling::AtSubject *sub) override;

protected:
   virtual void MakeTab(TEveWindowSlot *slot) override;

   void DrawHistELossVRange();
   void DrawHistELossVRange(AtTrack::BraggCurve braggCurve);

   virtual void UpdatePatternEventElements() override;
   void UpdateTrackingEventElements();
   void UpdateFitMetadata();

   void PrintFittedTrackInfo(AtFittedTrack fittedTrack);
   void DrawBestFittingELoss(AtFittedTrack fittedTrack);

private:
   ClassDefOverride(AtTabBraggCurve, 1);
};
#endif
