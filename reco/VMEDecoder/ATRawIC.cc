#include <iostream>
#include "ATRawIC.hh"

ClassImp(ATRawIC);

ATRawIC::ATRawIC()
{   
  Initialize();
}


ATRawIC::~ATRawIC()
{
 
}

void ATRawIC::Initialize(){
     
    memset(fRawAdc, 0, sizeof(fRawAdc));
  
}

void ATRawIC::SetRawfADC(Int_t idx, Int_t val)       { fRawAdc[idx] = val;}

Int_t *ATRawIC::GetRawfADC()            { return fRawAdc; }
Int_t  ATRawIC::GetRawfADC(Int_t idx)   { return fRawAdc[idx]; }


