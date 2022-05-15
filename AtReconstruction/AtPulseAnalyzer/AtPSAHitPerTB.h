#ifndef ATPSAHITPERTB_H
#define ATPSAHITPERTB_H

#include "AtPSA.h"

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOverride
class AtEvent;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSAHitPerTB : public AtPSA {
protected:
   Int_t fIniTB{0};   //< First TB for charge integration
   Int_t fEndTB{512}; //< Last TB for charge integration

public:
   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSAHitPerTB>(*this); }
   void SetTBLimits(std::pair<Int_t, Int_t> limits);

   ClassDefOverride(AtPSAHitPerTB, 1)
};

#endif
