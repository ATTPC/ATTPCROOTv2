/*********************************************************************
*   ATTPC Pad Class	ATPad                                            *
*   Author: Y. Ayyad                                                 *
*   Log: 05-03-2015 19:24 JST                                        *
*                                                                    *
*                                                                    *
*********************************************************************/

#include <iostream>
#include "ATPad.hh"

ClassImp(ATPad);

ATPad::ATPad()
{
    fPadNum = -1;
    fPadXCoord = -9999;
    fPadYCoord = -9999;
    kIsAux = kFALSE;
    fAuxName = "noname";   
}

ATPad::ATPad(Int_t PadNum)
{
    Initialize();
    fPadNum = PadNum;
    fPadXCoord = -9999;
    fPadYCoord = -9999;
    kIsAux = kFALSE;
    fAuxName = "noname";
}


ATPad::~ATPad()
{

}

void ATPad::Initialize(){

    fIsPedestalSubtracted = 0;
    fMaxAdcIdx = 0;

    memset(fRawAdc, 0, sizeof(fRawAdc));
    memset(fAdc, 0, sizeof(fAdc));

}

void ATPad::SetValidPad(Bool_t val)               { kIsValid = val; }
void ATPad::SetPad(Int_t val)                     { fPadNum = val; }
void ATPad::SetRawADC(Int_t *val)                 { memcpy(fRawAdc,val, sizeof(fRawAdc)); }
void ATPad::SetRawADC(Int_t idx, Int_t val)       { fRawAdc[idx] = val;}
void ATPad::SetPedestalSubtracted(Bool_t val)     { fIsPedestalSubtracted = val; }
//void ATPad::SetGainCalibrated(Bool_t val)       { fIsGainCalibrated = val; }//TODO
void ATPad::SetMaxADCIdx(Int_t val)               { fMaxAdcIdx = val; }
void ATPad::SetADC(Double_t *val)                 { memcpy(fAdc, val, sizeof(fAdc)); }
void ATPad::SetADC(Int_t idx, Double_t val)       { fAdc[idx] = val; }
void ATPad::SetPadXCoord(Double_t val)            { fPadXCoord = val;}
void ATPad::SetPadYCoord(Double_t val)            { fPadYCoord = val;}
void ATPad::SetIsAux(Bool_t val)                  { kIsAux = val;}
void ATPad::SetAuxName(std::string val)           { fAuxName = val;}

Float_t ATPad::GetPadXCoord()                     { return fPadXCoord;}
Float_t ATPad::GetPadYCoord()                     { return fPadYCoord;}

Bool_t ATPad::GetValidPad()                       { return kIsValid;}
Bool_t ATPad::IsAux()                             { return kIsAux;}

std::string ATPad::GetAuxName()                   { return fAuxName;}

ATPad &ATPad::operator= (ATPad right)
{
  Initialize();

  fPadNum = right.GetPadNum();


  memcpy(fRawAdc, right.GetRawADC(), sizeof(fRawAdc));
  memcpy(fAdc, right.GetADC(), sizeof(fAdc));

  fMaxAdcIdx = right.GetMaxADCIdx();

  fIsPedestalSubtracted = right.IsPedestalSubtracted();
  kIsAux = right.IsAux();
  //fIsGainCalibrated = right.IsGainCalibrated();//todo

  return *this;
}

 Int_t  ATPad::GetPadNum()             { return fPadNum; }
 Int_t *ATPad::GetRawADC()             { return fRawAdc; }
 Int_t  ATPad::GetRawADC(Int_t idx)    { return fRawAdc[idx]; }
 Int_t  ATPad::GetMaxADCIdx()          { return fMaxAdcIdx; }
Bool_t  ATPad::IsPedestalSubtracted()  { return fIsPedestalSubtracted; }
//Bool_t  ATPad::IsGainCalibrated()     { return fIsGainCalibrated; }//TODO

Double_t *ATPad::GetADC()
{
  if (!fIsPedestalSubtracted) {
    std::cout << "== Pedestal subtraction is not done!" << std::endl;

    return 0;
  }

  return fAdc;
}

Double_t ATPad::GetADC(Int_t idx)
{
  if (!fIsPedestalSubtracted) {
    std::cout << "== Pedestal subtraction is not done!" << std::endl;

    return -4;
  }

  return fAdc[idx];
}
