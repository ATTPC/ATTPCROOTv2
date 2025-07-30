#ifndef ATPSAHITPERTB_H
#define ATPSAHITPERTB_H

#include "AtPSA.h"

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOverride

#include <memory>  // for make_unique, unique_ptr
#include <utility> // for pair

class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSAHitPerTB : public AtPSA {
protected:
   Int_t fIniTB{0};   //< First TB for charge integration
   Int_t fEndTB{512}; //< Last TB for charge integration

   // Whether or not to replace all trace integrals by the total trace integral.
   Bool_t fReplaceTraceIntegral{true};

public:
   virtual HitVector AnalyzePad(AtPad *pad) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSAHitPerTB>(*this); }

   void SetTBLimits(std::pair<Int_t, Int_t> limits);
   void SetReplaceTraceIntegral(Bool_t value) { fReplaceTraceIntegral = value; }

   ClassDefOverride(AtPSAHitPerTB, 2)
};

#endif
