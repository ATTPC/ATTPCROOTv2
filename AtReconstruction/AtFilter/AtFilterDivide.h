#ifndef ATFILTERDIVIDE_H
#define ATFILTERDIVIDE_H
#include "AtFilter.h"

#include <Rtypes.h>

class AtPad;
class AtRawEvent;
struct AtPadReference;

/**
 * Example filter to divide the signal by some amount specified at run time
 * @ingroup RawFilters
 */
class AtFilterDivide : public AtFilter {

private:
   Double_t fDivisor;

public:
   void SetDivisor(Double_t divisor);
   Double_t GetDivisor() { return fDivisor; }

   virtual void Init() override;
   virtual void InitEvent(AtRawEvent *event) override;
   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;
   virtual bool IsGoodEvent() override;
};

#endif //#ifndef ATFILTERDIVIDE_H
