#ifndef AtFITTER_H
#define AtFITTER_H

#include "AtDigiPar.h"
#include "AtTrack.h"
#include "AtEvent.h"
#include "AtPatternEvent.h"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

namespace AtFITTER {

class AtFitter : public TObject {

public:
   AtFitter();
   virtual ~AtFitter();
   // virtual std::vector<AtTrack> GetFittedTrack() = 0;
   virtual bool FitTracks(AtPatternEvent &patternEvent) = 0;
   virtual void Init()=0;
  
protected:
   FairLogger *fLogger; ///< logger pointer
   AtDigiPar *fPar;     ///< parameter container
   std::tuple<Double_t,Double_t>   GetMomFromBrho(Double_t A, Double_t Z, Double_t brho); ///< Returns momentum (in GeV) from Brho assuming M (amu) and Z;  

   ClassDef(AtFitter, 1)
};

} // namespace AtFITTER

#endif
