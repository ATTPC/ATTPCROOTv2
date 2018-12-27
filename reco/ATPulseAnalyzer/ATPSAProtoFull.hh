#ifndef ATPSAPROTOFULL_H
#define ATPSAPROTOFULL_H

#include "ATPSA.hh"
#include "TH1F.h"

// ROOT classes
#include "TSpectrum.h"

class ATPSAProtoFull : public ATPSA
{
  public:
    ATPSAProtoFull();
    ~ATPSAProtoFull();

    void Analyze(ATRawEvent *rawEvent, ATEvent *event);

  ClassDef(ATPSAProtoFull, 1)
};

#endif
