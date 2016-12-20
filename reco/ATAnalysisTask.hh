#ifndef ATANALYSISTASK_H
#define ATANALYSISTASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

#include "TF1.h"
#include "TGraph.h"

// ATTPCROOT classes
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATProtoEventAna.hh"
#include "ATTrackingEventAna.hh"
#include "ATDigiPar.hh"
#include "ATHoughSpaceCircle.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpace.hh"
#include "ATAnalysis.hh"
#include "ATProtoAnalysis.hh"
#include "ATTrackingAnalysis.hh"
#include "ATRansac.hh"

#include "AtTpcMap.h"
#include "TH2Poly.h"

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__


// ROOT classes
#include "TClonesArray.h"

class ATAnalysisTask : public FairTask {
  public:
    ATAnalysisTask();
    ~ATAnalysisTask();

    typedef boost::multi_array<double,3> multiarray;
    typedef multiarray::index index;
    multiarray fAtPadCoord;

    void SetPersistence(Bool_t value = kTRUE);
    void SetPhiReco(); //Hough Space is calculated for the prototype after sorting the hits by quadrant. Phi Reconstruction is
    void SetFullScale();
    void SetELossPar(std::vector<Double_t> par[10]);
    void SetEtoRParameters(std::vector<Double_t> (&parRtoE)[10]);
    void AddParticle(std::vector<std::pair<Int_t,Int_t>> ptcl);
    void SetEnableMap();
    void SetMap(Char_t const *map);
    void SetSimpleMode(); // Simple tracking analysis without considering the vertex (RANSAC+Tracking Analysis)
    // needed prior to this mode of the task
    void SetHoughDist(Double_t value);
    void SetUpperLimit(Double_t value);
    void SetLowerLimit(Double_t value);
    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;
    //TClonesArray *fEventHArray;
    TClonesArray *fProtoEventHArray;
    TClonesArray *fProtoEventAnaArray;
    TClonesArray *fTrackingEventAnaArray;
    TClonesArray *fHoughArray;
    TClonesArray *fRansacArray;
    //TClonesArray *fAnalysisArray;

    ATProtoAnalysis     *fProtoAnalysis;
    ATTrackingAnalysis  *fTrackingAnalysis;
    ATHoughSpaceLine    *fHoughSpace;
    ATProtoEvent        *fProtoevent;
    ATRANSACN::ATRansac *fRansac;



    ATDigiPar *fPar;
    Bool_t fIsPersistence;
    Bool_t fIsPhiReco;
    Bool_t fIsFullScale;
    Double_t fHoughDist;
    Double_t fUpperLimit;
    Double_t fLowerLimit;

    Int_t fRunNum;
    Int_t fInternalID;

    TF1 *fHoughFit[4];
    TGraph *fHitPatternFilter[4];
    TF1 *fFitResult[4];

    std::vector<Double_t> fELossPar[10];
    std::vector<Double_t> fEtoRPar[10];
    std::vector<std::pair<Int_t,Int_t>> fParticleAZ;

    AtTpcMap *fAtMapPtr;
    Char_t const *fMap;
    TH2Poly *fPadPlane;
    Bool_t fIsEnableMap;
    Bool_t fIsSimpleMode;

  ClassDef(ATAnalysisTask, 1);
};

#endif
