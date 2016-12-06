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
#include "AtTpcMap.h"
#include "TH2Poly.h"

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

//Boost
#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

class ATAnalysis : public TObject
{
     public:
        ATAnalysis();
        virtual ~ATAnalysis();

        typedef boost::multi_array<double,3> multiarray;
        typedef multiarray::index index;

       //virtual void Analyze(ATProtoEvent* protoevent,ATProtoEventAna* protoeventAna,ATHoughSpaceLine* houghspace,TF1 *(&HoughFit)[4],TGraph *(&HitPatternFilter)[4],TF1 *(&FitResult)[4])=0;
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
       Double_t fThetaPad;
       Double_t fThetaLorentz;
       Double_t fThetaRot;
       Double_t fPressure;
       Double_t fMaxRange;
       Double_t fGain;
       Double_t fCoefL;
       Double_t fCoefT;

       ClassDef(ATAnalysis, 1);

};

#endif
