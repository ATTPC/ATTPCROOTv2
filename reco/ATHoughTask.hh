#ifndef ATHOUGHTASK_H
#define ATHOUGHTASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// ATTPCROOT classes
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATDigiPar.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpace.hh"

#include "AtTpcMap.h"

// ROOT classes
#include "TClonesArray.h"

class ATHoughTask : public FairTask {
  public:
    ATHoughTask();
    ~ATHoughTask();


    void SetPersistence(Bool_t value = kTRUE);
    void SetThreshold(Double_t threshold);
    void SetLinearHough();
    void SetCircularHough();
    void SetPhiReco(); //Hough Space is calculated for the prototype after sorting the hits by quadrant. Phi Reconstruction is
    // needed prior to this mode of the task
    void SetRadiusThreshold(Float_t value);
    void SetHoughThreshold(Double_t value);
    void SetEnableMap();
    void SetMap(Char_t const *map);

    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);
    //virtual void FinishEvent();

  private:
    FairLogger *fLogger;

    ATDigiPar *fPar;

    TClonesArray *fEventHArray;
    TClonesArray *fProtoEventHArray;
    TClonesArray *fHoughArray;

    ATHoughSpace *fHoughSpace;
    ATEvent *fEvent;
    ATProtoEvent *fProtoevent;

    Bool_t fIsPersistence;
    Bool_t fIsLinear;
    Bool_t fIsCircular;
    Bool_t fIsPhiReco; //Hough Space applied after Phi Reconstruction to divide hits in quadrants
    Bool_t fIsEnableMap;


    Double_t fThreshold;
    Double_t fRadThreshold;
    Double_t fHoughThreshold;

    Int_t fInternalID;

    AtTpcMap *fAtMapPtr;
    Char_t const *fMap;

  ClassDef(ATHoughTask, 1);
};

#endif
