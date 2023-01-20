#ifndef ATSIDEBARADDON_H
#define ATSIDEBARADDON_H

#include "AtSidebarFrames.h"

#include <Rtypes.h>
#include <TGFrame.h>
#include <TGLabel.h>
#include <TGNumberEntry.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>

class AtSidebarAddon : public AtVerticalSidebarFrame {
protected:
   std::map<std::string, TGNumberEntry *> fNumbers;

public:
   AtSidebarAddon(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                  Pixel_t back = GetDefaultFrameBackground())
      : AtVerticalSidebarFrame(p, w, h, options, back)
   {
   }

   void AddIntBox(std::string title, std::string function, int min = 0, int max = 1);

   void FillFrame() override {};

   ClassDefOverride(AtSidebarAddon, 1);
};

#endif
