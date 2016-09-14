#ifndef ATANALYSIS_H
#define ATANALYSIS_H

#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATProtoEventAna.hh"
#include "ATProtoQuadrant.hh"
#include "ATHoughSpace.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpaceCircle.hh"
#include "ATHit.hh"

#include "TF1.h"
#include "TGraph.h"


// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

// STL
#include <vector>
#include <algorithm>

// ROOT classes
#include "TClonesArray.h"

class ATAnalysis : public TObject
{
     public:
        ATAnalysis();
        virtual ~ATAnalysis();

       virtual void Analyze(ATProtoEvent* protoevent,ATProtoEventAna* protoeventAna,ATHoughSpaceLine* houghspace,TF1 *(&HoughFit)[4],TGraph *(&HitPatternFilter)[4],TF1 *(&FitResult)[4])=0;
     protected:
       FairLogger *fLogger;      ///< logger pointer
       ATDigiPar *fPar;          ///< parameter container

       Int_t fNumTbs;            ///< the number of time buckets used in taking data
       Int_t fTBTime;            ///< time duration of a time bucket in ns
       Int_t fEntTB;
       Double_t fDriftVelocity;  ///< drift velocity of electron in cm/us
       Double_t fMaxDriftLength; ///< maximum drift length in mm
       Double_t fZk;             //Relative position of micromegas-cathode

       Int_t fThreshold;         ///< threshold of ADC value
       Double_t fBField;
       Double_t fEField;
       Double_t fTiltAng;
       TVector3 fLorentzVector;
       Int_t fTB0;

       ClassDef(ATAnalysis, 1);

};

#endif
