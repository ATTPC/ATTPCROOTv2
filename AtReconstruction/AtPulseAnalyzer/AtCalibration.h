#ifndef AtCALIBRAtION_H
#define AtCALIBRAtION_H

#include <Rtypes.h>
#include <TObject.h>
#include <TString.h>

#include <array>

class TBuffer;
class TClass;
class TMemberInspector;

using trace = std::array<Double_t, 512>;

class AtCalibration : public TObject {
protected:
   TString fGainFile;
   TString fJitterFile;

   trace fGadc{};
   trace fGnewadc{};
   trace fJadc{};
   trace fJnewadc{};

   Bool_t fIsGainCalibrated{false};
   Bool_t fIsJitterCalibrated{false};

   std::array<Double_t, 10240> fGainCalib{};
   std::array<Double_t, 10240> fJitterCalib{};

   Int_t fPadNum{};

public:
   AtCalibration() = default;
   ~AtCalibration() = default;

   void SetGainFile(TString gainFile);
   void SetJitterFile(TString jitterFile);

   const trace &CalibrateGain(const trace &adc, Int_t padNum);
   const trace &CalibrateJitter(const trace &adc, Int_t padNum);

   Bool_t IsGainFile();
   Bool_t IsJitterFile();

   ClassDef(AtCalibration, 3);
};
#endif
