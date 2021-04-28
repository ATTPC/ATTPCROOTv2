#ifndef ATFILTERDIVIDE_H
#define ATFILTERDIVIDE_H
// Example filter to divide the signal by some amount specified at run time
#include "AtFilter.h"

class AtFilterDivide : public AtFilter {

private:
   Double_t fDivisor;

public:
   void SetDivisor(Double_t divisor);
   Double_t GetDivisor() { return fDivisor; }

   // Called at the init stage of the AtFilterTask
   virtual void Init() override;
   // Called on each pad if no baseline subtraction
   virtual void Filter(Int_t *trace) override;
   // Called on each pad if baseline subtraction
   virtual void Filter(Double_t *trace) override;
};

#endif //#ifndef ATFILTERDIVIDE_H
