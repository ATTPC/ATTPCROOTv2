#ifndef AtPSATBAVG_H
#define AtPSATBAVG_H

#include "AtPSA.h"

#include <Rtypes.h> // for Bool_t, THashConsistencyHolder, ClassDefOverride

#include <limits>
#include <memory> // for make_unique, unique_ptr

class AtEvent;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief Simple max finding PSA method.
 *
 */
class AtPSATBAvg : public AtPSA {
private:
   Int_t fTBtoAvg{5};
   Double_t fMaxThreshold{std::numeric_limits<Double_t>::max()};

public:
   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSATBAvg>(*this); }

   void SetNumTBToAvg(Int_t num) { fTBtoAvg = num; }
   void SetMaxThreshold(Double_t max) { fMaxThreshold = max; }

private:
   Double_t getThreshold(int padSize);

   ClassDefOverride(AtPSATBAvg, 1)
};

#endif
