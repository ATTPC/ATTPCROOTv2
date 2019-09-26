#ifndef ATCALIBRATION_H
#define ATCALIBRATION_H

#include <fstream>
#include <iostream>

#include "TObject.h"
#include "TString.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

class ATCalibration : public TObject {
  public:
      ATCalibration();
      ~ATCalibration();

      void SetGainFile(TString gainFile);
      void SetJitterFile(TString jitterFile);

      Double_t *CalibrateGain(Double_t adc[512], Int_t padNum);
      Double_t *CalibrateJitter(Double_t adc[512], Int_t padNum);

      Bool_t IsGainFile();
      Bool_t IsJitterFile();


    protected:
      TString fGainFile;
      TString fJitterFile;

      Double_t fGadc[512];
      Double_t fGnewadc[512];
      Double_t fJadc[512];
      Double_t fJnewadc[512];

      Bool_t fIsGainCalibrated;
      Bool_t fIsJitterCalibrated;

      Double_t fGainCalib[10240];
      Double_t fJitterCalib[10240];

      Int_t fPadNum;


    ClassDef(ATCalibration, 1);
};
#endif
