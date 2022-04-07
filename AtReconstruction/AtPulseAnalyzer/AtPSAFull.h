#ifndef AtPSAFULL_H
#define AtPSAFULL_H

#include "AtPSA.h"

// ROOT classes

class AtPSAFull : public AtPSA {
public:
   AtPSAFull() = default;
   ~AtPSAFull() = default;

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

   ClassDefOverride(AtPSAFull, 1)
};

#endif
