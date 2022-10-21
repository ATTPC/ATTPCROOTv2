// Example filter to divide the signal by some amount specified at run time
#include "AtFilterDivide.h"

#include "AtPad.h"

class AtRawEvent;
struct AtPadReference;

void AtFilterDivide::SetDivisor(Double_t div)
{
   fDivisor = div;
}

void AtFilterDivide::Init() {}

void AtFilterDivide::InitEvent(AtRawEvent *event) {}

void AtFilterDivide::Filter(AtPad *pad, AtPadReference *padReference)
{
   for (int i = 0; i < 512; ++i) {
      pad->SetRawADC(i, pad->GetRawADC(i) / fDivisor);
      if (pad->IsPedestalSubtracted())
         pad->SetADC(i, pad->GetADC(i) / fDivisor);
   }
}

bool AtFilterDivide::IsGoodEvent()
{
   return true;
}
