#ifndef ATPSAFULL_H
#define ATPSAFULL_H

#include "ATPSA.hh"

// ROOT classes
#include "TSpectrum.h"

class ATPSAFull : public ATPSA{
public:
  ATPSAFull();
  ~ATPSAFull();
  
  void Analyze(ATRawEvent *rawEvent, ATEvent *event);
  
  ClassDef(ATPSAFull, 1)
};

#endif
