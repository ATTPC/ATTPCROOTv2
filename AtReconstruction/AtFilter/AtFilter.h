#ifndef ATFILTER_H
#define ATFILTER_H

class TClonesArray;
class AtRawEvent;
class AtPad;
struct AtPadReference;

/**
 * @brief Interface for filters that can be applied to the raw signal traces.
 * @defgroup RawFilters Raw Event Fitlers
 */
class AtFilter {

public:
   virtual ~AtFilter() = default;

   /**
    * @brief Called at the init stage of the AtFilterTask.
    */
   virtual void Init() = 0;

   /**
    * @brief Construct output event from input event array.
    *
    * Called to construct and return the output event in the array from the input event
    * This only needs to be overriden if you are doing something unusual and cannot just
    * copy the input event
    */
   virtual AtRawEvent *ConstructOutputEvent(TClonesArray *outputEventArray, AtRawEvent *inputEvent);

   /**
    * @brief Called once for each event at the start of the Exec phase.
    */
   virtual void InitEvent(AtRawEvent *inputEvent) = 0;

   /**
    * @brief Called to filter each pad.
    *
    * Optional padRef currently only used by SCA filter (10/20/22).
    *
    * @param[in] pad Pad to filter.
    * @param[in[ padRefernece optional parameter only needed when when filtering FPN channels.
    */
   virtual void Filter(AtPad *pad, AtPadReference *padReference = nullptr) = 0;

   /// Called at the end of an event. Returns if filtering was successful.
   virtual bool IsGoodEvent() = 0;
};

#endif //#ifndef ATFILTER_H
