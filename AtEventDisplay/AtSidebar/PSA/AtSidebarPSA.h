#ifndef ATSIDEBARPSA_H
#define ATSIDEBARPSA_H
#include "AtPSA.h"
#include "AtSidebarAddon.h"
#include "AtSidebarFrames.h"

#include <Rtypes.h>
#include <TGFrame.h>
#include <TGLabel.h>
#include <TGNumberEntry.h>

#include <string>

class AtSidebarPSA : public AtSidebarAddon {
protected:
   AtPSA *fPSA;

   const std::string fThreshold{"Threshold"};

public:
   AtSidebarPSA(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarAddon(p, w, h, options, back)
   {
   }

   void SetPSA(AtPSA *psa) { fPSA = psa; }

   void SetThreshold();

   void FillFrame() override;

   ClassDefOverride(AtSidebarPSA, 1);
};

#endif
