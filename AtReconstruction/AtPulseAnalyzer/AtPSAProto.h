#ifndef AtPSAPROTO_H
#define AtPSAPROTO_H

#include "AtPSA.h"

#include <Rtypes.h> // for THashConsistencyHolder, Bool_t, ClassDefOverride
class AtEvent;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSAProto : public AtPSA {
public:
   AtPSAProto() = default;
   ~AtPSAProto() = default;

   void SetBackGroundSuppression();

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

private:
   Bool_t fBackGroundSuppression;

   ClassDefOverride(AtPSAProto, 2)
};

#endif
