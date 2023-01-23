#ifndef ATSIDEBARPSADECONV_H
#define ATSIDEBARPSADECONV_H
#include "AtSidebarPSA.h" // for AtSidebarPSA

#include <Rtypes.h> // for THashConsistencyHolder, UInt_t, ClassDefOv...

#include <GuiTypes.h> // for Pixel_t

#include <string> // for allocator, string
class TBuffer;
class TClass;
class TGWindow;
class TMemberInspector;

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
