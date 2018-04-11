#ifndef ATPRA_H
#define ATPRA_H

#include "ATDigiPar.hh"
#include "ATTrack.hh"
#include "ATEvent.hh"
#include "ATPatternEvent.hh"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"


namespace ATPATTERN{

      class ATPRA : public TObject
      {

          public:

            virtual ~ATPRA();
            virtual std::vector<ATTrack> GetTrackCand() = 0;
            virtual bool FindTracks(ATEvent &event, ATPatternEvent *patternEvent) = 0;

          protected:
            FairLogger *fLogger;      ///< logger pointer
            ATDigiPar *fPar;          ///< parameter container




            ClassDef(ATPRA, 1)
      };

}//namespace

#endif
