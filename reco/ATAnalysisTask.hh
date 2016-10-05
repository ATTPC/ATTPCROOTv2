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
#include "ATDigiPar.hh"
#include "ATHoughSpaceCircle.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpace.hh"
#include "ATAnalysis.hh"
#include "ATProtoAnalysis.hh"



// ROOT classes
#include "TClonesArray.h"

class ATAnalysisTask : public FairTask {
  public:
    ATAnalysisTask();
    ~ATAnalysisTask();

    void SetPersistence(Bool_t value = kTRUE);
    void SetPhiReco(); //Hough Space is calculated for the prototype after sorting the hits by quadrant. Phi Reconstruction is
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
    TClonesArray *fHoughArray;
    //TClonesArray *fAnalysisArray;

    ATProtoAnalysis   *fProtoAnalysis;
    ATHoughSpaceLine  *fHoughSpace;
    ATProtoEvent      *fProtoevent;
    TClonesArray      *fRansacArray;


    ATDigiPar *fPar;
    Bool_t fIsPersistence;
    Bool_t fIsPhiReco;
    Double_t fHoughDist;
    Double_t fUpperLimit;
    Double_t fLowerLimit;

    Int_t fRunNum;
    Int_t fInternalID;

    TF1 *fHoughFit[4];
    TGraph *fHitPatternFilter[4];
    TF1 *fFitResult[4];

  ClassDef(ATAnalysisTask, 1);
};

#endif
