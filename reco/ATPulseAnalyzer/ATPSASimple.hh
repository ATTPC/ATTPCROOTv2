#ifndef ATPSASIMPLE_H
#define ATPSASIMPLE_H

#include "ATPSA.hh"

class ATPSASimple:public ATPSA {
 public:
    ATPSASimple();
    ~ATPSASimple();

    void Analyze(ATRawEvent * rawEvent, ATEvent * event) override;

     ClassDefOverride(ATPSASimple, 2);

};

#endif
