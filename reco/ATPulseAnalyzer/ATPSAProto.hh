#ifndef ATPSAPROTO_H
#define ATPSAPROTO_H

#include "ATPSA.hh"

class ATPSAProto:public ATPSA {
 public:
    ATPSAProto();
    ~ATPSAProto();

    void SetBackGroundSuppression();

    void Analyze(ATRawEvent * rawEvent, ATEvent * event) override;

 private:
     Bool_t fBackGroundSuppression;

 ClassDefOverride(ATPSAProto, 2)};

#endif
