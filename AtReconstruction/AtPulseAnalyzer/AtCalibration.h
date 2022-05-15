#ifndef AtCALIBRATION_H
#define AtCALIBRATION_H

#include <Rtypes.h>
#include <TString.h>

#include <array>

using trace = std::array<Double_t, 512>;

/**
 * AtFilter should be prefered to this class which is setup for just AT-TPC data.
 */
class AtCalibration {
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
   void SetGainFile(TString gainFile);
   void SetJitterFile(TString jitterFile);

   const trace &CalibrateGain(const trace &adc, Int_t padNum);
   const trace &CalibrateJitter(const trace &adc, Int_t padNum);

   Bool_t IsGainFile();
   Bool_t IsJitterFile();
};
#endif
