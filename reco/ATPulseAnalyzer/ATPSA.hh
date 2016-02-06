#ifndef ATPSA_H
#define ATPSA_H

#include "ATRawEvent.hh"
#include "ATPad.hh"
#include "ATEvent.hh"
#include "ATHit.hh"
#include "ATDigiPar.hh"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

// STL
#include <vector>

// ROOT classes
#include "TClonesArray.h"
#include "TSpectrum.h"
#include "TVector3.h"
#include "TMath.h"

class ATPSA
{
  public:
    ATPSA();
    virtual ~ATPSA();

    //! Setting threshold
    void SetThreshold(Int_t threshold);
    void SetBackGroundSuppression();
    void SetPeakFinder();
    void SetMaxFinder();
    void SetBaseCorrection(Bool_t value);
    void SetTimeCorrection(Bool_t value);

    virtual void Analyze(ATRawEvent *rawEvent, ATEvent *event) = 0;

  protected:
    FairLogger *fLogger;      ///< logger pointer
    ATDigiPar *fPar;          ///< parameter container

    Bool_t fBackGroundSuppression;
    Bool_t fIsPeakFinder;
    Bool_t fIsMaxFinder;
    Bool_t fIsBaseCorr;
    Bool_t fIsTimeCorr;


    Int_t fPadPlaneX;         ///< pad plane size x in mm
    Int_t fPadSizeX;          ///< pad size x in mm
    Int_t fPadSizeZ;          ///< pad size y in mm
    Int_t fPadRows;           ///< number of total pad rows
    Int_t fPadLayers;         ///< number of total pad layers

    Int_t fNumTbs;            ///< the number of time buckets used in taking data
    Int_t fTBTime;            ///< time duration of a time bucket in ns
    Double_t fDriftVelocity;  ///< drift velocity of electron in cm/us
    Double_t fMaxDriftLength; ///< maximum drift length in mm

    Int_t fThreshold;         ///< threshold of ADC value
    Double_t fBField;
    Double_t fEField;
    Double_t fTiltAng;
    TVector3 fLorentzVector;
    Int_t fTB0;


    Double_t CalculateX(Double_t row);      ///< Calculate x position in mm. This returns the center position of given pad row.
    Double_t CalculateZ(Double_t peakIdx);  ///< Calculate z position in mm using the peak index.
    Double_t CalculateY(Double_t layer);    ///< Calculate y position in mm. This returns the center position of given pad layer.

    Double_t CalculateXCorr(Double_t xval, Int_t Tbx);
    Double_t CalculateYCorr(Double_t yval, Int_t Tby);
    Double_t CalculateZCorr(Double_t zval, Int_t Tbz);

    void CalcLorentzVector();



  ClassDef(ATPSA, 2)
};

#endif
