#ifndef ATPSFILTER_H
#define ATPSFILTER_H

#include "ATPSA.hh"

// ROOT classes
#include "TSpectrum.h"

class ATPSAFilter : public ATPSA
{
  public:
    ATPSAFilter();
    ~ATPSAFilter();

    void Analyze(ATRawEvent *rawEvent, ATEvent *event);

  private:
    //TSpectrum *fPeakFinder;  /// TSpectrum object

  ClassDef(ATPSAFilter, 1)
};

#endif
