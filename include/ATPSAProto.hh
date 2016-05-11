#ifndef ATPSAPROTO_H
#define ATPSAPROTO_H

#include "ATPSA.hh"
#include "TH1F.h"

// ROOT classes
#include "TSpectrum.h"

class ATPSAProto : public ATPSA
{
  public:
    ATPSAProto();
    ~ATPSAProto();

    void Analyze(ATRawEvent *rawEvent, ATEvent *event);

  private:
    //TSpectrum *fPeakFinder;  /// TSpectrum object



  ClassDef(ATPSAProto, 1)
};

#endif
