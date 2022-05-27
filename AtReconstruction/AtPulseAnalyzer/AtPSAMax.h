#ifndef AtPSAMAX_H
#define AtPSAMAX_H

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
 * @brief Simple max finding PSA method.
 *
 *
 *
 */
class AtPSAMax : public AtPSA {

private:
   Bool_t fIsTimeCorr{false};

public:
   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSAMax>(*this); }

   void SetTimeCorrection(Bool_t value) { fIsTimeCorr = value; }

private:
   Double_t getThreshold(int padSize);
   bool shouldSaveHit(double charge, double threshold, int tb);
   Double_t getTBCorr(std::array<Double_t, 512> &trace, int maxAdcIdx);

   ClassDefOverride(AtPSAMax, 1)
};

#endif
