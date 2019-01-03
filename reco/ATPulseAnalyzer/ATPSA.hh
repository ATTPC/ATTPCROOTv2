#ifndef ATPSA_H
#define ATPSA_H

#include "ATRawEvent.hh"
#include "ATPad.hh"
#include "ATEvent.hh"
#include "ATHit.hh"
#include "ATDigiPar.hh"
#include "ATCalibration.hh"

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
    //void SetAuxChannel(std::vector<Int_t> AuxCh);

    void SetMeanK(Int_t value); //Number of neighbors
    void SetStddevMulThresh(Double_t value);
    void SetGainCalibration(TString gainFile);
    void SetJitterCalibration(TString jitterFile);
    void SetTBLimits(std::pair<Int_t,Int_t> limits);

    virtual void Analyze(ATRawEvent *rawEvent, ATEvent *event) = 0;

  protected:
    FairLogger *fLogger;      ///< logger pointer
    ATDigiPar *fPar;          ///< parameter container

    Bool_t fBackGroundSuppression;
    Bool_t fIsPeakFinder;
    Bool_t fIsMaxFinder;
    Bool_t fIsBaseCorr;
    Bool_t fIsTimeCorr;

    Bool_t fIsGainCalibrated;
    Bool_t fIsJitterCalibrated;

    ATCalibration *fCalibration;

    Int_t fPadPlaneX;         ///< pad plane size x in mm
    Int_t fPadSizeX;          ///< pad size x in mm
    Int_t fPadSizeZ;          ///< pad size y in mm
    Int_t fPadRows;           ///< number of total pad rows
    Int_t fPadLayers;         ///< number of total pad layers

    Int_t fNumTbs;            ///< the number of time buckets used in taking data
    Int_t fTBTime;            ///< time duration of a time bucket in ns
    Int_t fEntTB;
    Int_t fIniTB;             /// First TB for charge integration
    Int_t fEndTB;             /// Last TB for charge integration
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
    Int_t fMeanK;
    Double_t fStdDev;
    //std::vector<Int_t> fAuxChannels; //Auxiliary external channels


    Double_t CalculateX(Double_t row);      ///< Calculate x position in mm. This returns the center position of given pad row.
    Double_t CalculateZ(Double_t peakIdx);  ///< Calculate z position in mm using the peak index.
    Double_t CalculateY(Double_t layer);    ///< Calculate y position in mm. This returns the center position of given pad layer.

    Double_t CalculateZGeo(Double_t peakIdx);

    Double_t CalculateXCorr(Double_t xval, Int_t Tbx);
    Double_t CalculateYCorr(Double_t yval, Int_t Tby);
    Double_t CalculateZCorr(Double_t zval, Int_t Tbz);

    void CalcLorentzVector();

    TVector3 RotateDetector(Double_t x,Double_t y,Double_t z,Int_t tb);



  ClassDef(ATPSA, 3)
};

#endif
