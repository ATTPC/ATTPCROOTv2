#ifndef AtPSASIMPLE_H
#define AtPSASIMPLE_H

#include "AtPSA.h"

class AtPSASimple : public AtPSA {
public:
   AtPSASimple();
   ~AtPSASimple();

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

   ClassDefOverride(AtPSASimple, 2);
};

#endif
