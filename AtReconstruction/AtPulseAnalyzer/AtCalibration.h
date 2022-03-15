#ifndef AtCALIBRAtION_H
#define AtCALIBRAtION_H

#include <fstream>
#include <iostream>

#include "TObject.h"
#include "TString.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

using trace = std::array<Double_t, 512>;

class AtCalibration : public TObject {
public:
   AtCalibration();
   ~AtCalibration();

   void SetGainFile(TString gainFile);
   void SetJitterFile(TString jitterFile);

   const trace &CalibrateGain(const trace &adc, Int_t padNum);
   const trace &CalibrateJitter(const trace &adc, Int_t padNum);

   Bool_t IsGainFile();
   Bool_t IsJitterFile();

protected:
   TString fGainFile;
   TString fJitterFile;

   Double_t fGadc[512];
   trace fGnewadc;
   Double_t fJadc[512];
   trace fJnewadc;

   Bool_t fIsGainCalibrated;
   Bool_t fIsJitterCalibrated;

   Double_t fGainCalib[10240];
   Double_t fJitterCalib[10240];

   Int_t fPadNum;

   ClassDef(AtCalibration, 2);
};
#endif
