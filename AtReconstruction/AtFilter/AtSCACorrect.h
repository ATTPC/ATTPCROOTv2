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

   Bool_t fDoBaseline;
   Bool_t fDoPhase;
   Bool_t fUseChanZero;

   TString fBaseAugName;
   TString fPhaseAugName;

public:
   AtSCACorrect(AtMapPtr map, TString filename, TString eventName, TString baselineAug, TString phaseAug);
   AtSCACorrect(AtMapPtr map, RawEventPtr rawEvent, TString baselineAug, TString phaseAug);

   virtual void Init() override {}
   virtual void InitEvent(AtRawEvent *) override {}
   virtual bool IsGoodEvent() override { return true; }

   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;

   void DoBaseline(Bool_t val) { fDoBaseline = val; }
   void DoPhase(Bool_t val) { fDoPhase = val; }

   void UseChannelZero() { fUseChanZero = true; }

private:
   void removeBaseline(AtPad *pad, AtPadReference *padReference);
   void removePhase(AtPad *pad, AtPadReference *padReference);
   AtPad *getMatchingPad(AtPad *pad, AtPadReference *ref, AtRawEvent *event);
};

#endif // #ifndef ATSCACORRECT_H
