#ifndef AtPATTERNMODIFICATION_H
#define AtPATTERNMODIFICATION_H

#include "AtEvent.h"
#include "AtPatternEvent.h"
#include "AtRawEvent.h"

class AtPatternModification : public TObject {
protected:
   // Pointer to the AtPatternEvent to be modified.
   AtPatternEvent *fPatternEvent{nullptr};

   // Pointers to AtRawEvent and AtEvent that may be used or not in the modification.
   AtRawEvent *fRawEvent{nullptr};
   AtEvent *fEvent{nullptr};

public:
   AtPatternModification() = default;
   ~AtPatternModification() = default;

   virtual void ModifyPatternEvent() = 0;
   virtual void Init() = 0;

   // Mandatory to set.
   void SetPatternEvent(AtPatternEvent *patternEvent) { fPatternEvent = patternEvent; }

   // Optional to set.
   void SetRawEvent(AtRawEvent *rawEvent) { fRawEvent = rawEvent; }
   void SetEvent(AtEvent *event) { fEvent = event; }

   // Reset pointers.
   void Reset();

private:
   ClassDef(AtPatternModification, 1);
};

#endif
