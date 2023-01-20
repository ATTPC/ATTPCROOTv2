#ifndef ATSIDEBARPSAITERDECONV_H
#define ATSIDEBARPSAITERDECONV_H

#include "AtSidebarPSADeconv.h"
#include "AtSidebarAddon.h"
#include "AtSidebarFrames.h"

#include <Rtypes.h>
#include <TGFrame.h>
#include <TGLabel.h>
#include <TGNumberEntry.h>

#include <string>

class AtSidebarPSAIterDeconv : public AtSidebarPSADeconv {
protected:
   const std::string fIterations{"Iterations"};

public:
   AtSidebarPSAIterDeconv(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                          Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarPSADeconv(p, w, h, options, back)
   {
   }

   void SetIterations();

   void FillFrame() override;

   ClassDefOverride(AtSidebarPSAIterDeconv, 1);
};

#endif
