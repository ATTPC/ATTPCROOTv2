// Example filter to divide the signal by some amount specified at run time
#include "AtFilterDivide.h"

void AtFilterDivide::SetDivisor(Double_t div)
{
   fDivisor = div;
}

void AtFilterDivide::Init() {}

void AtFilterDivide::Filter(Int_t *trace)
{
   for (int i = 0; i < 512; ++i)
      trace[i] /= fDivisor;
}

void AtFilterDivide::Filter(Double_t *trace)
{
   for (int i = 0; i < 512; ++i)
      trace[i] /= fDivisor;
}
