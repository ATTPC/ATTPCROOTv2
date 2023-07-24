#ifndef ATSIDEBARPSATBAVG_H
#define ATSIDEBARPSATBAVG_H
#include "AtSidebarPSA.h" // for AtSidebarPSA

#include <Rtypes.h> // for THashConsistencyHolder, UInt_t, ClassDefOv...

#include <GuiTypes.h> // for Pixel_t

#include <string> // for allocator, string
class TBuffer;
class TClass;
class TGWindow;
class TMemberInspector;

class AtSidebarPSATBAvg : public AtSidebarPSA {
protected:
   const std::string fTBtoAvg{"TB to Average"};
   const std::string fMaxThreshold{"Max Threshold"};

public:
   AtSidebarPSATBAvg(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                     Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarPSA(p, w, h, options, back)
   {
   }

   void SetTBtoAvg();
   void SetMaxThreshold();

   void FillFrame() override;

   ClassDefOverride(AtSidebarPSATBAvg, 1);
};

#endif
