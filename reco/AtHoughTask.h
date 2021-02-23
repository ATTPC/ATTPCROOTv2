#ifndef AtHOUGHTASK_H
#define AtHOUGHTASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// AtTPCROOT classes
#include "AtEvent.h"
#include "AtProtoEvent.h"
#include "AtDigiPar.h"
#include "AtHoughSpaceLine.h"
#include "AtHoughSpace.h"

#include "AtTpcMap.h"
#include "TH2Poly.h"

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

// ROOT classes
#include "TClonesArray.h"

class AtHoughTask : public FairTask {
  public:
    AtHoughTask();
    ~AtHoughTask();

    typedef boost::multi_array<double,3> multiarray;
    typedef multiarray::index index;
    multiarray fAtPadCoord;

    void SetPersistence(Bool_t value = kTRUE);
    void SetThreshold(Double_t threshold);
    void SetLinearHough();
    void SetCircularHough();
    void SetPhiReco(); //Hough Space is calculated for the prototype after sorting the hits by quadrant. Phi Reconstruction is
    // needed prior to this mode of the task
    void SetRadiusThreshold(Float_t value);
    void SetHoughThreshold(Double_t value);
    void SetHoughDistance(Double_t value);
    void SetEnableMap();
    void SetMap(Char_t const *map);

    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);
    //virtual void FinishEvent();

  private:
    FairLogger *fLogger;

    AtDigiPar *fPar;

    TClonesArray *fEventHArray;
    TClonesArray *fProtoEventHArray;
    TClonesArray *fHoughArray;

    AtHoughSpace *fHoughSpace;
    AtEvent *fEvent;
    AtProtoEvent *fProtoevent;

    Bool_t fIsPersistence;
    Bool_t fIsLinear;
    Bool_t fIsCircular;
    Bool_t fIsPhiReco; //Hough Space applied after Phi Reconstruction to divide hits in quadrants
    Bool_t fIsEnableMap;


    Double_t fThreshold;
    Double_t fRadThreshold;
    Double_t fHoughThreshold;
    Double_t fHoughDistance;

    Int_t fInternalID;

    AtTpcMap *fAtMapPtr;
    Char_t const *fMap;
    TH2Poly *fPadPlane;

  ClassDef(AtHoughTask, 1);
};

#endif
