#ifndef ATSIDEBARPSADECONV_H
#define ATSIDEBARPSADECONV_H

#include "AtSidebarAddon.h"
#include "AtSidebarFrames.h"
#include "AtSidebarPSA.h"

#include <Rtypes.h>
#include <TGFrame.h>
#include <TGLabel.h>
#include <TGNumberEntry.h>

#include <string>

class AtSidebarPSADeconv : public AtSidebarPSA {
protected:
   const std::string fOrder{"Filter Order"};
   const std::string fCutoff{"Filter Cutoff"};

public:
   AtSidebarPSADeconv(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                      Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarPSA(p, w, h, options, back)
   {
   }

   void SetFilterOrder();
   void SetCutoffFreq();

   void FillFrame() override;

   ClassDefOverride(AtSidebarPSADeconv, 1);
};

#endif
