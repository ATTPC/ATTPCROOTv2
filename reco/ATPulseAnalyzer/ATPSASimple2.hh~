
#pragma once

#include "ATPSA.hh"

// ROOT classes
#include "TSpectrum.h"

class STPSASimple2 : public STPSA
{
  public:
    ATPSASimple2();
    ~ATPSASimple2();

    void Analyze(ATRawEvent *rawEvent, ATEvent *event);

  private:
    TSpectrum *fPeakFinder;  /// TSpectrum object

  ClassDef(ATPSASimple2, 1)
};
