#ifndef ATPSASIMPLE_H
#define ATPSASIMPLE_H

#include "ATPSA.hh"

class ATPSA;

class ATPSASimple : public ATPSA
{
  public:
    ATPSASimple();
    ~ATPSASimple();

    void Analyze(ATRawEvent *rawEvent, ATEvent *event);

  ClassDef(ATPSASimple, 1);
};

#endif
