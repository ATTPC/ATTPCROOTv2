#ifndef AtPSASIMPLE_H
#define AtPSASIMPLE_H

#include <Rtypes.h>

#include "AtPSA.h"

class AtEvent;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSASimple : public AtPSA {
public:
   AtPSASimple();
   ~AtPSASimple();

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

   ClassDefOverride(AtPSASimple, 2);
};

#endif
