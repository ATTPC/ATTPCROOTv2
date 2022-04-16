#ifndef AtPSAPROTOFULL_H
#define AtPSAPROTOFULL_H

#include "AtPSA.h"

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOverride
class AtEvent;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSAProtoFull : public AtPSA {
public:
   AtPSAProtoFull() = default;
   ~AtPSAProtoFull() = default;

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

   ClassDefOverride(AtPSAProtoFull, 1)
};

#endif
