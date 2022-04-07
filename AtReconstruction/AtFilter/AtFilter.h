#ifndef ATFILTER_H
#define ATFILTER_H
// Interface for filters that can be applied to the raw signal traces

class AtRawEvent;
class AtPad;

class AtFilter {

public:
   // Called at the init stage of the AtFilterTask
   virtual void Init() = 0;

   // Called once for each event at the start of the Exec phase
   virtual void InitEvent(AtRawEvent *event) = 0;

   // Called on each pad
   virtual void Filter(AtPad *pad) = 0;

   // Called at the end of an event. Returns if filtering was successful.
   virtual bool IsGoodEvent() = 0;
};

#endif //#ifndef ATFILTER_H
