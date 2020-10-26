#ifndef ATPSASimple2WITHFFT_H
#define ATPSASimple2WITHFFT_H

#include "ATPSA.hh"

// ROOT classes
#include "TSpectrum.h"

class ATPSASimple2withFFT : public ATPSA
{
  public:
    ATPSASimple2withFFT();
    ~ATPSASimple2withFFT();

    void Analyze(ATRawEvent *rawEvent, ATEvent *event);

  private:
    //TSpectrum *fPeakFinder;  /// TSpectrum object


  ClassDef(ATPSASimple2withFFT, 1)
};

#endif
