#ifndef ATFITTER_H
#define ATFITTER_H

#include "ATDigiPar.hh"
#include "ATTrack.hh"
#include "ATEvent.hh"
#include "ATPatternEvent.hh"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

namespace ATFITTER{

      class ATFitter : public TObject
      {

          public:

	    ATFitter();
            virtual ~ATFitter();
            //virtual std::vector<ATTrack> GetFittedTrack() = 0;
            virtual bool FitTracks(ATPatternEvent &patternEvent) = 0;

	  protected:
            FairLogger *fLogger;      ///< logger pointer
            ATDigiPar *fPar;          ///< parameter container
            

            ClassDef(ATFitter, 1)
      };

}//namespace


#endif


