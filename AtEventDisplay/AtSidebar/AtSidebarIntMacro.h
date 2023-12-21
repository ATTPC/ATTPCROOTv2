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

   int fMin;
   int fMax;
   int fStart;

public:
   /**
    * A sidebar object for an integer value whose value can be used in a user defined function.
    */
   AtSidebarIntMacro(const TGWindow *p = nullptr, int min = 0, int max = 10, int start = 0, UInt_t w = 1, UInt_t h = 1,
                     UInt_t options = 0, Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarAddon(p, w, h, options, back), fMin{min}, fMax{max}, fStart{start}
   {
   }
   ~AtSidebarIntMacro() {}

   void FillFrame() override;

   void RunFunction();

   /**
    * Set the lower and upper bounds for the integer.
    */
   void SetBounds(int min, int max)
   {
      fMin = min;
      fMax = max;
   }
   /**
    * Set the user defined function. The user function must be void and takes the value in as an int.
    */
   void SetFunction(MacroFunction function) { fFunction = std::move(function); }
   /**
    * Set the inital value of the integer.
    */
   void SetInitialValue(int value) { fStart = value; }
   /**
    * Set the label for the integer. The label will be displayed on the sidebar with the number. This label must be
    * unique.
    */
   void SetLabel(std::string label) { fLabel = label; }

   ClassDefOverride(AtSidebarIntMacro, 1);
};

#endif
