#ifndef AtPSASPECTRUM_H
#define AtPSASPECTRUM_H

#include "AtPSA.h"

#include <Rtypes.h> // for Bool_t, THashConsistencyHolder, ClassDefOverride

#include <array>  // for array
#include <memory> // for make_unique, unique_ptr

class AtHit;
class AtPad;
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
   HitVector AnalyzePad(AtPad *pad) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSASpectrum>(*this); }

   void SetBackGroundSuppression() { fBackGroundSuppression = true; }
   void SetBackGroundInterpolation() { fBackGroundInterp = true; }
   void SetTimeCorrection(Bool_t value) { fIsTimeCorr = value; }

protected:
   void subtractBackground(std::array<Double_t, 512> &adc);
   double calcTbCorrection(const std::array<Double_t, 512> &adc, int idxPeak);
   std::unique_ptr<AtHit> getHit(int idx);

   ClassDefOverride(AtPSASpectrum, 1)
};

#endif
