#ifndef ATSCACORRECT_H
#define ATSCACORRECT_H

#include "AtFilter.h"

#include <TString.h>

#include <memory> // for shared_ptr

class AtPad;
class AtMap;
class AtRawEvent;
struct AtPadReference;

/**
 * A filter to perform baseline subtraction using the average baseline and average phase effect stored as
 * AtRawEvent objects in a separate file or files.
 * @ingroup RawFilters
 */
class AtSCACorrect : public AtFilter {
private:
   using AtMapPtr = std::shared_ptr<AtMap>;
   using RawEventPtr = std::unique_ptr<AtRawEvent>;
   AtMapPtr fMap;

   RawEventPtr fRawEvent;

   bool fDoBaseline{false};
   bool fDoPhase{false};
   bool fUseChanZero{false};

   TString fBaseAugName;
   TString fPhaseAugName;

public:
   /**
    * Baseline and phase effects can be stored in a AtRawEvent object in a file.
    *
    * filename: Name of the file
    * eventName: Name of the AtRawEvent object
    * baselineAug: The string used to identify the AtPad augment the baseline is stored in
    * phaseAug: The string used to identify the AtPad augment the phase effect is stored in
    */
   AtSCACorrect(AtMapPtr map, TString filename, TString eventName, TString baselineAug, TString phaseAug);

   /**
    * Baseline and phase effects can be stored in a AtRawEvent object with a unique pointer supplied to the constructor
    *
    * baselineAug: The string used to identify the AtPad augment the baseline is stored in
    * phaseAug: The string used to identify the AtPad augment the phase effect is stored in
    */
   AtSCACorrect(AtMapPtr map, RawEventPtr rawEvent, TString baselineAug, TString phaseAug);

   virtual void Init() override {}
   virtual void InitEvent(AtRawEvent *) override {}
   virtual bool IsGoodEvent() override { return true; }

   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;

   /**
    * Turns on and off the baseline correction.
    * Defaults to true. Set false to disable baseline correction.
    */
   void DoBaseline(Bool_t val) { fDoBaseline = val; }
   /**
    * Turns on and off the phase effect correction.
    * Defaults to true. Set false to disable phase effect correction.
    */
   void DoPhase(Bool_t val) { fDoPhase = val; }

   /**
    * Sets the use of channel zero for the given AGET chip for the baseline and phase effect insted of the actual
    * channel being corrected.
    */
   void UseChannelZero() { fUseChanZero = true; }

private:
   void removeBaseline(AtPad *pad, AtPadReference *padReference);
   void removePhase(AtPad *pad, AtPadReference *padReference);
   AtPad *getMatchingPad(AtPad *pad, AtPadReference *ref, AtRawEvent *event);
};

#endif // #ifndef ATSCACORRECT_H
