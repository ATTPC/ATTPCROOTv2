#ifndef ATPSAPROTOFULL_H
#define ATPSAPROTOFULL_H

#include "ATPSA.hh"

class ATPSAProtoFull:public ATPSA {
 public:
    ATPSAProtoFull();
    ~ATPSAProtoFull();

    void Analyze(ATRawEvent * rawEvent, ATEvent * event) override;

 ClassDefOverride(ATPSAProtoFull, 1)};

#endif
