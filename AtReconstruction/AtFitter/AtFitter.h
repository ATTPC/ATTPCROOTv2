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

protected:
   FairLogger *fLogger; ///< logger pointer
   AtDigiPar *fPar;     ///< parameter container

   ClassDef(AtFitter, 1)
};

} // namespace AtFITTER

#endif
