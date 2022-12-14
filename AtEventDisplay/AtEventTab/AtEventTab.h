#ifndef ATEVENTTAB_H
#define ATEVENTTAB_H

#include <Rtypes.h>

class TClass;
class AtRawEvent;
class AtEvent;

class AtEventTab {
private:
   bool fUsesEvent = false;
   bool fUsesRawEvent = false;

public:
   AtEventTab() = default;
   virtual ~AtEventTab() = default;

   virtual void Init() = 0;
   virtual void Reset() = 0;

   virtual void MakeTab() = 0;
   virtual void DrawEvent(AtRawEvent *rawEvent, AtEvent *event) = 0;
   virtual void DrawPad(Int_t padNum) = 0;

   void SetTaskNumber(Int_t taskNum) { fTaskNumber = taskNum; }

protected:
   Int_t fTaskNumber;

   ClassDef(AtEventTab, 1)
};

#endif
