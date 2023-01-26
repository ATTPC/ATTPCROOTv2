#ifndef ATSIDEBARINFOMACRO_H
#define ATSIDEBARINFOMACRO_H

#include "AtDataObserver.h"
#include "AtSidebarAddon.h"
#include "AtViewerManagerSubject.h" // for AtTreeEntry

#include <Rtypes.h> // for THashConsistencyHolder, UInt_t, ClassDef...

#include <GuiTypes.h> // for Pixel_t

#include <functional>
#include <string>
#include <utility> // for move

class TBuffer;
class TClass;
class TGWindow;
class TMemberInspector;
namespace DataHandling {
class AtSubject;
}

class AtSidebarInfoMacro : public AtSidebarAddon, public DataHandling::AtObserver {
private:
   using MacroFunction = std::function<std::string()>;
   MacroFunction fFunction;

   DataHandling::AtTreeEntry &fEntryNumber;

   std::string fLabel;

public:
   AtSidebarInfoMacro(DataHandling::AtTreeEntry &entryNumber, const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1,
                      UInt_t options = 0, Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarAddon(p, w, h, options, back), fEntryNumber(entryNumber)
   {
      fEntryNumber.Attach(this);
   }
   ~AtSidebarInfoMacro() { fEntryNumber.Detach(this); }

   void SetFunction(MacroFunction function) { fFunction = std::move(function); }
   void SetLabel(std::string label) { fLabel = label; }

   void Update(DataHandling::AtSubject *sub) override;

   void FillFrame() override;

   ClassDefOverride(AtSidebarInfoMacro, 1);
};

#endif
