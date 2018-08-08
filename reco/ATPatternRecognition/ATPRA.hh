#ifndef ATPRA_H
#define ATPRA_H

#include "ATDigiPar.hh"
#include "ATTrack.hh"
#include "ATEvent.hh"
#include "ATPatternEvent.hh"
#include "ATRansac.hh"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

//ROOT
#include "TVirtualFitter.h"


namespace ATPATTERN{

      class ATPRA : public TObject
      {

          public:

            virtual ~ATPRA();
            virtual std::vector<ATTrack> GetTrackCand() = 0;
            virtual bool FindTracks(ATEvent &event, ATPatternEvent *patternEvent) = 0;

            void SetTrackInitialParameters(ATTrack& track);

          protected:
            FairLogger *fLogger;      ///< logger pointer
            ATDigiPar *fPar;          ///< parameter container

            void SetTrackCurvature(ATTrack& track);
            

            ClassDef(ATPRA, 1)
      };

}//namespace

Double_t fitf(Double_t *x,Double_t *par);

#endif
