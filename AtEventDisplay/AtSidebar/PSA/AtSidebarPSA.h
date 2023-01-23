#ifndef ATSIDEBARPSA_H
#define ATSIDEBARPSA_H
#include "AtSidebarAddon.h" // for AtSidebarAddon

#include <Rtypes.h> // for THashConsistencyHolder, UInt_t, ClassDef...

#include <GuiTypes.h> // for Pixel_t

#include <string> // for allocator, string
class AtPSA;
class TBuffer;
class TClass;
class TGWindow;
class TMemberInspector;

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
