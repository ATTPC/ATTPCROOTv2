#ifndef ATTABCANVAS_H
#define ATTABCANVAS_H
#include <Rtypes.h>  // for Int_t, THashConsistencyHolder, ClassDefOverride
#include <TString.h> // for TString

#include <AtTabBase.h>

class TBuffer;
class TClass;
class TEveWindowSlot;
class TMemberInspector;
class TCanvas;

/**
 * @brief Abstract class for a tab composed of a single TCanvas.
 *
 * Optionally can be subdivded into multiple canvases.
 * We don't use TRootEmbeddedCanvas because we may want the toolbar
 */
class AtTabCanvas : public AtTabBase {
protected:
   TCanvas *fCanvas{nullptr};
   Int_t fRows, fCols;

public:
   AtTabCanvas(TString name, Int_t rows = 1, Int_t cols = 1) : AtTabBase(name), fRows(rows), fCols(cols) {}

protected:
   void MakeTab(TEveWindowSlot *slot) override;
   void UpdateCanvas();

   ClassDefOverride(AtTabCanvas, 1)
};

#endif
