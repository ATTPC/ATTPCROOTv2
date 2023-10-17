#ifndef ATSIDEBARINTMACRO_H
#define ATSIDEBARINTMACRO_H

#include "AtSidebarAddon.h"

#include <Rtypes.h> // for THashConsistencyHolder, UInt_t, ClassDef...

#include <GuiTypes.h> // for Pixel_t

#include <functional>
#include <string>
#include <utility> // for move

class TBuffer;
class TClass;
class TGWindow;
class TMemberInspector;

class AtSidebarIntMacro : public AtSidebarAddon {
private:
   using MacroFunction = std::function<void(int)>;
   MacroFunction fFunction;

   std::string fLabel;

   int fMin{0};
   int fMax{10};
   int fStart{0};

public:
   AtSidebarIntMacro(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                     Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarAddon(p, w, h, options, back)
   {
   }
   ~AtSidebarIntMacro() {}

   void FillFrame() override;

   void RunFunction();

   void SetBounds(int min, int max)
   {
      fMin = min;
      fMax = max;
   }
   void SetFunction(MacroFunction function) { fFunction = std::move(function); }
   void SetInitialValue(int value) { fStart = value; }
   void SetLabel(std::string label) { fLabel = label; }

   ClassDefOverride(AtSidebarIntMacro, 1);
};

#endif
