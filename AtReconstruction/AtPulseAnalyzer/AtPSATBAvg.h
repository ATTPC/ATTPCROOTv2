#ifndef AtPSATBAVG_H
#define AtPSATBAVG_H

#include "AtPSA.h"

#include <Rtypes.h> // for Bool_t, THashConsistencyHolder, ClassDefOverride

#include <limits>
#include <memory> // for make_unique, unique_ptr

class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief Constructs a hit from averaged TBs.
 *
 */
class AtPSATBAvg : public AtPSA {
private:
   Int_t fTBtoAvg{5};
   Double_t fMaxThreshold{std::numeric_limits<Double_t>::max()};

public:
   HitVector AnalyzePad(AtPad *pad) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSATBAvg>(*this); }

   void SetNumTBToAvg(Int_t num) { fTBtoAvg = num; }
   void SetMaxThreshold(Double_t max) { fMaxThreshold = max; }

   ClassDefOverride(AtPSATBAvg, 1)
};

#endif
