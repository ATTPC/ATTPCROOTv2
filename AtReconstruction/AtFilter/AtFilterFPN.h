#ifndef ATFILTERFPN_H
#define ATFILTERFPN_H
#include "AtFilterSubtraction.h"
#include "AtPad.h" // for AtPad

#include <Rtypes.h> // for Int_t
class AtRawEvent;
struct AtPadReference;

class AtFilterFPN : public AtFilterSubtraction {
protected:
   bool fAverageAgets{false};

   virtual bool isValidPad(const AtPad &pad) override;
   virtual int getAsad(const AtPadReference &ref) override;

public:
   AtFilterFPN(AtMapPtr map, bool averageAgets, Int_t numCoBos = 10);

   virtual void InitEvent(AtRawEvent *event) override;
};
#endif //#ifndef ATFILTERFPN_H
