#ifndef ATFILTER_H
#define ATFILTER_H
// Interface for filters that can be applied to the raw signal traces

class TClonesArray;
class AtRawEvent;
class AtPad;

class AtFilter {

public:
   virtual ~AtFilter() = default;

   // Called at the init stage of the AtFilterTask
   virtual void Init() = 0;

   // Called to construct and return the output event in the array from the input event
   // This only needs to be overriden if you are doing something unusual and cannot just
   // copy the input event
   virtual AtRawEvent *ConstructOutputEvent(TClonesArray *outputEventArray, AtRawEvent *inputEvent);

   // Called once for each event at the start of the Exec phase
   virtual void InitEvent(AtRawEvent *inputEvent) = 0;

   // Called on each pad
   virtual void Filter(AtPad *pad) = 0;

   // Called at the end of an event. Returns if filtering was successful.
   virtual bool IsGoodEvent() = 0;
};

#endif //#ifndef ATFILTER_H
