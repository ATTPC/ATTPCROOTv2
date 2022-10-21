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

   RawEventPtr fBaseline;
   RawEventPtr fPhase;

public:
   AtSCACorrect(AtMapPtr map, TString baselineFilename, TString baselineEventName, TString phaseFilename,
                TString phaseEventName);
   AtSCACorrect(AtMapPtr map, RawEventPtr baseline, RawEventPtr phase);

   virtual void Init() override {}
   virtual void InitEvent(AtRawEvent *) override {}
   virtual bool IsGoodEvent() override { return true; }

   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;

private:
   void removeBaseline(AtPad *pad, AtPadReference *padReference);
   void removePhase(AtPad *pad, AtPadReference *padReference);
   AtPad *getMatchingPad(AtPad *pad, AtPadReference *ref, AtRawEvent *event);
};

#endif //#ifndef ATSCACORRECT_H
