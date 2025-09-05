#ifndef AtPATTERNMODIFICATION_H
#define AtPATTERNMODIFICATION_H

#include "AtEvent.h"
#include "AtPatternEvent.h"
#include "AtRawEvent.h"

class AtPatternModification {
public:
   AtPatternModification() = default;
   virtual ~AtPatternModification() = default;

   virtual void Init() = 0;

   /**
    * Function that unpacks the AtPatternEvent and iterates over the track candidates in order to apply the
    * GetModifiedTrack to each one.
    */
   virtual void
   ModifyPatternEvent(AtPatternEvent *patternEvent, AtRawEvent *rawEvent = nullptr, AtEvent *event = nullptr);

protected:
   /**
    * Actually implements ths track modification.
    */
   virtual AtTrack GetModifiedTrack(const AtTrack &track, AtRawEvent *rawEvent = nullptr, AtEvent *event = nullptr) = 0;
};

#endif
