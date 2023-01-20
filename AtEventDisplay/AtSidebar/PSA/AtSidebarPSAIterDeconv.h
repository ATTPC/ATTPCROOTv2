#ifndef ATSIDEBARPSAITERDECONV_H
#define ATSIDEBARPSAITERDECONV_H
#include "AtPSAIterDeconv.h"
#include "AtSidebarFrames.h"

#include <Rtypes.h>
#include <TGFrame.h>
#include <TGLabel.h>
#include <TGNumberEntry.h>

class AtSidebarPSAIterDeconv : public AtVerticalSidebarFrame {
protected:
   AtPSAIterDeconv *fPSA;

   TGHorizontalFrame *fThresholdFrame{nullptr};
   TGLabel *fThresholdLabel{nullptr};
   TGNumberEntry *fThresholdEntry{nullptr};

   TGHorizontalFrame *fOrderFrame{nullptr};
   TGLabel *fOrderLabel{nullptr};
   TGNumberEntry *fOrderEntry{nullptr};

   TGHorizontalFrame *fCutoffFrame{nullptr};
   TGLabel *fCutoffLabel{nullptr};
   TGNumberEntry *fCutoffEntry{nullptr};

   TGHorizontalFrame *fIterationsFrame{nullptr};
   TGLabel *fIterationsLabel{nullptr};
   TGNumberEntry *fIterationsEntry{nullptr};

public:
   AtSidebarPSAIterDeconv(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                          Pixel_t back = GetDefaultFrameBackground())
      : AtVerticalSidebarFrame(p, w, h, options, back)
   {
   }

   void SetPSA(AtPSAIterDeconv *psa) { fPSA = psa; }

   void SetThreshold() { fPSA->SetThreshold(fThresholdEntry->GetNumberEntry()->GetIntNumber()); }
   void SetFilterOrder() { fPSA->SetFilterOrder(fOrderEntry->GetNumberEntry()->GetIntNumber()); }
   void SetCuttoffFreq() { fPSA->SetCutoffFreq(fCutoffEntry->GetNumberEntry()->GetIntNumber()); }
   void SetIterations() { fPSA->SetIterations(fIterationsEntry->GetNumberEntry()->GetIntNumber()); }

   void FillFrame() override;

   // ClassDef(AtSidebarPSAIterDeconv, 1);
};

#endif
