#ifndef ATTABBASE_H
#define ATTABBASE_H

#include <Rtypes.h>

class AtTabInfo;
class TClass;

class AtTabBase {
public:
   AtTabBase();
   virtual ~AtTabBase() = default;

   void Init();
   virtual void InitTab() = 0;
   virtual void Reset() = 0;
   void Update();
   virtual void UpdateTab() = 0;


   virtual void MakeTab() = 0;
   virtual void DrawTree() = 0;
   virtual void DrawEvent() = 0;
   virtual void DrawPad(Int_t padNum) = 0;

   void SetTabNumber(Int_t tabNum) { fTabNumber = tabNum; }

protected:
   Int_t fTabNumber;
   AtTabInfo *fTabInfo;

   ClassDef(AtTabBase, 1)
};

#endif
