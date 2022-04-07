#ifndef AtCALIBRAtION_H
#define AtCALIBRAtION_H

#include <Rtypes.h>
#include <TObject.h>
#include <TString.h>
#include <array>

class TBuffer;
class TClass;
class TMemberInspector;

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

using trace = std::array<Double_t, 512>;

class AtCalibration : public TObject {
protected:
   TString fGainFile;
   TString fJitterFile;

   trace fGadc;
   trace fGnewadc;
   trace fJadc;
   trace fJnewadc;

   Bool_t fIsGainCalibrated;
   Bool_t fIsJitterCalibrated;

   std::array<Double_t, 10240> fGainCalib;
   std::array<Double_t, 10240> fJitterCalib;

   Int_t fPadNum;

public:
   AtCalibration();
   ~AtCalibration();

   void SetGainFile(TString gainFile);
   void SetJitterFile(TString jitterFile);

   const trace &CalibrateGain(const trace &adc, Int_t padNum);
   const trace &CalibrateJitter(const trace &adc, Int_t padNum);

   Bool_t IsGainFile();
   Bool_t IsJitterFile();

   ClassDef(AtCalibration, 3);
};
#endif
