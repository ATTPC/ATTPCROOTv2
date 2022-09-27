#ifndef ATFILTERFPN_H
#define ATFILTERFPN_H
#include "AtFilterSubtraction.h"
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
