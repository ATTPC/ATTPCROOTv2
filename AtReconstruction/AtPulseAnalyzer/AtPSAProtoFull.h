#ifndef AtPSAPROTOFULL_H
#define AtPSAPROTOFULL_H

#include "AtPSA.h"

class AtPSAProtoFull : public AtPSA {
public:
   AtPSAProtoFull() = default;
   ~AtPSAProtoFull() = default;

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

   ClassDefOverride(AtPSAProtoFull, 1)
};

#endif
