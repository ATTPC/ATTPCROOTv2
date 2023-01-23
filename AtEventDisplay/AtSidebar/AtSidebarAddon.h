#ifndef ATSIDEBARADDON_H
#define ATSIDEBARADDON_H

#include "AtSidebarFrames.h" // for AtVerticalSidebarFrame

#include <Rtypes.h> // for THashConsistencyHolder, UInt_t, ClassDe...

#include <GuiTypes.h> // for Pixel_t

#include <map>    // for map
#include <string> // for string
class TBuffer;
class TClass;
class TGNumberEntry;
class TGWindow;
class TMemberInspector;

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

   void FillFrame() override{};

   ClassDefOverride(AtSidebarAddon, 1);
};

#endif
