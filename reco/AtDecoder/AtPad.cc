/*********************************************************************
*   AtTPC Pad Class	AtPad                                            *
*   Author: Y. Ayyad                                                 *
*   Log: 05-03-2015 19:24 JST                                        *
*                                                                    *
*                                                                    *
*********************************************************************/

#include <iostream>
#include "AtPad.h"

ClassImp(AtPad);

AtPad::AtPad()
{
    fPadNum = -1;
    fSizeID = -1000;
    fPadXCoord = -9999;
    fPadYCoord = -9999;
    kIsAux = kFALSE;
    fAuxName = "noname";
}

AtPad::AtPad(Int_t PadNum)
{
    Initialize();
    fPadNum = PadNum;
    fSizeID = -1000;
    fPadXCoord = -9999;
    fPadYCoord = -9999;
    kIsAux = kFALSE;
    fAuxName = "noname";
}


AtPad::~AtPad()
{

}

void AtPad::Initialize(){

    fIsPedestalSubtracted = 0;
    fMaxAdcIdx = 0;

    memset(fRawAdc, 0, sizeof(fRawAdc));
    memset(fAdc, 0, sizeof(fAdc));

}

void AtPad::SetValidPad(Bool_t val)               { kIsValid = val; }
void AtPad::SetPad(Int_t val)                     { fPadNum = val; }
void AtPad::SetRawADC(Int_t *val)                 { memcpy(fRawAdc,val, sizeof(fRawAdc)); }
void AtPad::SetRawADC(Int_t idx, Int_t val)       { fRawAdc[idx] = val;}
void AtPad::SetPedestalSubtracted(Bool_t val)     { fIsPedestalSubtracted = val; }
void AtPad::SetSizeID(Int_t val)                     { fSizeID = val; }
//void AtPad::SetGainCalibrated(Bool_t val)       { fIsGainCalibrated = val; }//TODO
void AtPad::SetMaxADCIdx(Int_t val)               { fMaxAdcIdx = val; }
void AtPad::SetADC(Double_t *val)                 { memcpy(fAdc, val, sizeof(fAdc)); }
void AtPad::SetADC(Int_t idx, Double_t val)       { fAdc[idx] = val; }
void AtPad::SetPadXCoord(Double_t val)            { fPadXCoord = val;}
void AtPad::SetPadYCoord(Double_t val)            { fPadYCoord = val;}
void AtPad::SetIsAux(Bool_t val)                  { kIsAux = val;}
void AtPad::SetAuxName(std::string val)           { fAuxName = val;}

Float_t AtPad::GetPadXCoord()                     { return fPadXCoord;}
Float_t AtPad::GetPadYCoord()                     { return fPadYCoord;}

Bool_t AtPad::GetValidPad()                       { return kIsValid;}
Bool_t AtPad::IsAux()                             { return kIsAux;}

std::string AtPad::GetAuxName()                   { return fAuxName;}

AtPad &AtPad::operator= (AtPad right)
{
  Initialize();

  fPadNum = right.GetPadNum();

  fSizeID = right.GetSizeID();


  memcpy(fRawAdc, right.GetRawADC(), sizeof(fRawAdc));
  memcpy(fAdc, right.GetADC(), sizeof(fAdc));

  fMaxAdcIdx = right.GetMaxADCIdx();

  fIsPedestalSubtracted = right.IsPedestalSubtracted();
  kIsAux = right.IsAux();
  //fIsGainCalibrated = right.IsGainCalibrated();//todo

  return *this;
}

 Int_t  AtPad::GetPadNum()             { return fPadNum; }
 Int_t  AtPad::GetSizeID()             { return fSizeID; }
 Int_t *AtPad::GetRawADC()             { return fRawAdc; }
 Int_t  AtPad::GetRawADC(Int_t idx)    { return fRawAdc[idx]; }
 Int_t  AtPad::GetMaxADCIdx()          { return fMaxAdcIdx; }
Bool_t  AtPad::IsPedestalSubtracted()  { return fIsPedestalSubtracted; }
//Bool_t  AtPad::IsGainCalibrated()     { return fIsGainCalibrated; }//TODO

Double_t *AtPad::GetADC()
{
  if (!fIsPedestalSubtracted) {
    std::cout << "== Pedestal subtraction is not done!" << std::endl;

    return 0;
  }

  return fAdc;
}

Double_t AtPad::GetADC(Int_t idx)
{
  if (!fIsPedestalSubtracted) {
    std::cout << "== Pedestal subtraction is not done!" << std::endl;

    return -4;
  }

  return fAdc[idx];
}
