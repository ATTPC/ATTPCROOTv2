#ifndef ATSIDEBARADDON_H
#define ATSIDEBARADDON_H

#include "AtSidebarFrames.h" // for AtVerticalSidebarFrame

#include <Rtypes.h>  // for THashConsistencyHolder, UInt_t, ClassDe...
#include <TString.h> // for TString

#include <GuiTypes.h> // for Pixel_t

#include <map>    // for map
#include <string> // for string
class TGLabel;
class TBuffer;
class TClass;
class TGNumberEntry;
class TGWindow;
class TMemberInspector;

/**
 * A base class for creating addons to the Event Viewer. Contains functions for adding different types of sidebar items.
 * See AtSidebarPSA for an example of the implementation.
 */
class AtSidebarAddon : public AtVerticalSidebarFrame {
protected:
   std::map<std::string, TGNumberEntry *> fNumbers;
   std::map<std::string, TGLabel *> fStrings;

public:
   AtSidebarAddon(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                  Pixel_t back = GetDefaultFrameBackground())
      : AtVerticalSidebarFrame(p, w, h, options, back)
   {
   }

   /**
    * Adds a text box for an string that updates with SetInfoString.
    * label is used as the title for the text box and to index the string stored by the text box.
    */
   void AddInfoBox(std::string label);

   /**
    * Adds a text box for an itneger that triggers a function defined in the derived class.
    * label is used as the title for the text box and to index the number stored by the text box.
    * min and max are inlclusive bounds.
    */
   void AddIntBox(std::string label, std::string function, int min = 0, int max = 1);

   /**
    * Get the string managed by an AddInfoBox indexed by label.
    */
   TString GetInfoString(std::string label);

   /**
    * Get the integer managed by an AddIntBox indexed by label.
    */
   Long_t GetIntNumber(std::string label);

   /**
    * Set the string managed by an AddInfoBox indexed by label.
    */
   void SetInfoString(std::string label, TString value);

   /**
    * Set the integer managed by an AddIntBox indexed by label.
    */
   void SetIntNumber(std::string label, Long_t value);

   void FillFrame() override{};

   ClassDefOverride(AtSidebarAddon, 1);
};

#endif
