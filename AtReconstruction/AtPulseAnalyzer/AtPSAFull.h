#ifndef AtPSAFULL_H
#define AtPSAFULL_H

#include "AtPSA.h"

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOverride

#include <memory> // for make_unique, unique_ptr

class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSAFull : public AtPSA {
public:
   AtPSAFull() = default;
   ~AtPSAFull() = default;

   HitVector AnalyzePad(AtPad *pad) override;
   std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSAFull>(*this); }

   ClassDefOverride(AtPSAFull, 1)
};

#endif
