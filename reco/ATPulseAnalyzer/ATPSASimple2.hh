#ifndef ATPSASIMPLE2_H
#define ATPSASIMPLE2_H

#include "ATPSA.hh"

// ROOT classes
#include "TSpectrum.h"

class ATPSASimple2 : public ATPSA
{
  public:
    ATPSASimple2();
    ~ATPSASimple2();

    void Analyze(ATRawEvent *rawEvent, ATEvent *event);

  private:
    //TSpectrum *fPeakFinder;  /// TSpectrum object


  ClassDef(ATPSASimple2, 1)
};

#endif
