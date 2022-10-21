#ifndef ATFILTERZERO_H
#define ATFILTERZERO_H

#include "AtFilter.h"

#include <cstdlib> // IWYU pragma: keep
class AtRawEvent;
class AtPad;
struct AtPadReference;

/**
 * Class to look through traces and attempt to fill in any missing data by taking the average of the two
 * neighboring timebuckets. It will do a sanity check and if the difference between the two neighboring time
 * buckets is above a certain threshold, then it will just copy the value of the TB just before the missing data.
 * @ingroup RawFilters
 */
class AtFilterZero : public AtFilter {

private:
   double fThreshold{1000};

public:
   void SetThreshold(double val) { fThreshold = std::abs(val); }

   virtual void Init() override{};
   virtual void InitEvent(AtRawEvent *) override{};
   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;
   virtual bool IsGoodEvent() override { return true; }

private:
   void fillMissingData(AtPad *pad, int start, int stop);
   void fillMissingDataLine(AtPad *pad, int start, int stop);
};

#endif //#ifndef ATFILTERZERO_H
