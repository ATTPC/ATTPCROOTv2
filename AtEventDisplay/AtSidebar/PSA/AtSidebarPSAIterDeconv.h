#ifndef ATSIDEBARPSAITERDECONV_H
#define ATSIDEBARPSAITERDECONV_H
#include "AtSidebarPSADeconv.h" // for AtSidebarPSADeconv

#include <Rtypes.h> // for THashConsistencyHolder, UInt_t, Clas...

#include <GuiTypes.h> // for Pixel_t

#include <string> // for allocator, string
class TBuffer;
class TClass;
class TGWindow;
class TMemberInspector;

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
