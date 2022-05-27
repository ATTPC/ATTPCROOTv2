#ifndef AtPSASPECTRUM_H
#define AtPSASPECTRUM_H

#include "AtPSA.h"

#include <Rtypes.h> // for Bool_t, THashConsistencyHolder, ClassDefOverride

#include <array>  // for array
#include <memory> // for make_unique, unique_ptr

class AtEvent;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief PSA method using TSpectrum.
 *
 * Can use TSpectrum both to identify peaks, and to do a background subtraction
 */
class AtPSASpectrum : public AtPSA {

private:
   Bool_t fBackGroundSuppression{false}; //< Flag to pass to TSpectrum
   Bool_t fBackGroundInterp{false};
   Bool_t fIsTimeCorr{false};

public:
   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSASpectrum>(*this); }

   void SetBackGroundSuppression() { fBackGroundSuppression = true; }
   void SetBackGroundInterpolation() { fBackGroundInterp = true; }
   void SetTimeCorrection(Bool_t value) { fIsTimeCorr = value; }

protected:
   void subtractBackground(std::array<Double_t, 512> &adc);
   double calcTbCorrection(const std::array<Double_t, 512> &adc, int idxPeak);
   ClassDefOverride(AtPSASpectrum, 1)
};

#endif
