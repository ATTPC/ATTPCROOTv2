/*********************************************************************
 *   AtTPC Pad Class	AtPad                                            *
 *   Author: Y. Ayyad            				                     *
 *   Log: 05-03-2015 19:24 JST					                     *
 *   Adapted from SPiRITROOT STPad by G. Jhang                        *
 *								                                     *
 *********************************************************************/

#ifndef AtPAD_H
#define AtPAD_H

#include "TObject.h"
#include "TROOT.h"

class AtPad : public TObject {
private:
   Int_t fPadNum;
   Int_t fSizeID;
   Float_t fPadXCoord;
   Float_t fPadYCoord;
   Bool_t kIsValid;
   Int_t fRawAdc[512];
   Int_t fMaxAdcIdx;

   Bool_t fIsPedestalSubtracted;
   // Bool_t fIsGainCalibrated;
   Double_t fAdc[512];
   Bool_t kIsAux;
   std::string fAuxName;

public:
   AtPad();
   AtPad(Int_t PadNum);
   ~AtPad();

   void Initialize();
   void SetValidPad(Bool_t val = kTRUE);
   void SetPad(Int_t val);
   void SetRawADC(Int_t *val);
   void SetRawADC(Int_t idx, Int_t val);
   void SetMaxADCIdx(Int_t val);
   void SetSizeID(Int_t val);

   void SetPedestalSubtracted(Bool_t val = kTRUE);
   // void SetGainCalibrated(Bool_t val = kTRUE); //TODO
   void SetADC(Double_t *val);
   void SetADC(Int_t idx, Double_t val);
   void SetPadXCoord(Double_t val);
   void SetPadYCoord(Double_t val);
   void SetIsAux(Bool_t val);
   void SetAuxName(std::string val);

   AtPad &operator=(AtPad right);

   Bool_t IsPedestalSubtracted();
   Bool_t IsAux();

   Int_t GetPadNum();
   Int_t *GetRawADC();
   Int_t GetRawADC(Int_t idx);
   Int_t GetMaxADCIdx();
   Bool_t GetValidPad();
   std::string GetAuxName();
   Int_t GetSizeID();

   Double_t *GetADC();
   Double_t GetADC(Int_t idx);

   Float_t GetPadXCoord();
   Float_t GetPadYCoord();


   ClassDef(AtPad, 1);
};

#endif
