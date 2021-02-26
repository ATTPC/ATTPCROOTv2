#include <iostream>
#include "AtRawIC.h"

ClassImp(AtRawIC);

AtRawIC::AtRawIC()
{
   Initialize();
}

AtRawIC::~AtRawIC() {}

void AtRawIC::Initialize()
{

   memset(fRawAdc, 0, sizeof(fRawAdc));
}

void AtRawIC::SetRawfADC(Int_t idx, Int_t val)
{
   fRawAdc[idx] = val;
}

Int_t *AtRawIC::GetRawfADC()
{
   return fRawAdc;
}
Int_t AtRawIC::GetRawfADC(Int_t idx)
{
   return fRawAdc[idx];
}
