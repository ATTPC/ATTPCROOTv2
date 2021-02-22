#ifndef ATPSAFULL_H
#define ATPSAFULL_H

#include "ATPSA.hh"

// ROOT classes

class ATPSAFull:public ATPSA {
 public:
    ATPSAFull();
    ~ATPSAFull();

    void Analyze(ATRawEvent * rawEvent, ATEvent * event) override;

 ClassDefOverride(ATPSAFull, 1)};

#endif
