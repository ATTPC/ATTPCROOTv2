#ifndef ATTABBASE_H
#define ATTABBASE_H

#include "AtTabInfo.h" // IWYU pragma: keep

#include <Rtypes.h>

#include <memory>
class TClass;

class AtTabBase {
protected:
   Int_t fTabNumber{0};

   std::unique_ptr<AtTabInfo> fTabInfo{std::make_unique<AtTabInfo>()};

public:
   AtTabBase() = default;
   virtual ~AtTabBase() = default;

   void Init();
   /// Called second after Reset() in each Exec()
   void Update();

   AtTabInfo *GetTabInfo() { return fTabInfo.get(); }

   /// Called at the start of each Exec() so whenever the event changes
   virtual void Reset() = 0;

   virtual void MakeTab() = 0;
   virtual void DrawTree() = 0;
   /// Called at the end of each Exec()
   virtual void DrawEvent() = 0;
   virtual void DrawPad(Int_t padNum) = 0;

   void SetTabNumber(Int_t tabNum) { fTabNumber = tabNum; }

protected:
   /**
    * Responsible for creating the fTabInfo object that will be updated on each event.
    */
   virtual void InitTab() = 0;
   virtual void UpdateTab() = 0;

   ClassDef(AtTabBase, 1)
};

#endif
