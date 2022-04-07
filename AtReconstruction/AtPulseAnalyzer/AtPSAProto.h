#ifndef AtPSAPROTO_H
#define AtPSAPROTO_H

#include "AtPSA.h"

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
