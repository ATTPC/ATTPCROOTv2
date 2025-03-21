#ifndef ATPSAHITPERTBINREGION_H
#define ATPSAHITPERTBINREGION_H

#include "AtPSAHitPerTB.h"

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOverride

#include <memory>  // for make_unique, unique_ptr
#include <utility> // for pair

class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSAHitPerTBInRegion : public AtPSAHitPerTB {
public:
   virtual HitVector AnalyzePad(AtPad *pad) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSAHitPerTBInRegion>(*this); }
   void SetTBLimits(std::pair<Int_t, Int_t> limits) override;

   ClassDefOverride(AtPSAHitPerTBInRegion, 1)
};

#endif
