
#ifndef ATDIGIPAR_H
#define ATDIGIPAR_H

// FAIRROOT classes
#include "FairParGenericSet.h"
#include "FairParamList.h"
#include "FairLogger.h"
#include <iostream>
#include <fstream>

// SPiRITROOT classes
#include "ATGas.hh"

// ROOT classes
#include "TString.h"
#include "TSystem.h"

class ATDigiPar : public FairParGenericSet
{
  public:
    // Constructors and Destructors
    ATDigiPar(const Char_t *name, const Char_t *title, const Char_t *context);
    ~ATDigiPar();

    // Operators

    // Getters
    Int_t GetPadPlaneX();
    Int_t GetPadPlaneZ();
    Int_t GetPadSizeX();
    Int_t GetPadSizeZ();
    Int_t GetPadRows();
    Int_t GetPadLayers();
    Double_t GetAnodeWirePlaneY();
    Double_t GetGroundWirePlaneY();
    Double_t GetGatingWirePlaneY();
    ATGas *GetGas();
    Int_t GetNumTbs();
    Int_t GetTBTime();              ///< returns the time duration of a time bucket in given sampling time in ns.
    Double_t GetDriftVelocity();    ///< returns the drift velocity in cm/us.
    Double_t GetDriftLength();      ///< returns the drift length in mm
    Int_t GetYDivider();            ///< returns the slice divider
    virtual Bool_t getParams(FairParamList *paramList);

    TString GetFile(Int_t fileNum);

    Double_t GetBField();
    Double_t GetEField();
    Double_t GetTiltAngle();
    Int_t    GetTB0();
    Double_t GetThetaLorentz();
    Int_t    GetTBEntrance();
    Double_t GetZPadPlane();
    Double_t GetDensity();
    Double_t GetThetaPad();
    Double_t GetThetaRot();
    Double_t GetGasPressure();
    Double_t GetMaxRange();

    Double_t GetEIonize();
    Double_t GetCoefDiffusionTrans();
    Double_t GetCoefDiffusionLong();
    Double_t GetGain();


    // Setters
    virtual void putParams(FairParamList *paramList);

    // Main methods

  private:
    FairLogger *fLogger;

    ATGas *fGas;
    TString fGasFileName;

    Bool_t fInitialized;

    Int_t fPadPlaneX;
    Int_t fPadPlaneZ;
    Int_t fPadSizeX;
    Int_t fPadSizeZ;
    Int_t fPadRows;
    Int_t fPadLayers;
    Double_t fAnodeWirePlaneY;
    Double_t fGroundWirePlaneY;
    Double_t fGatingWirePlaneY;
    Double_t fEField;
    Int_t fNumTbs;
    Int_t fSamplingRate;
    Double_t fDriftVelocity;
    Double_t fDriftLength;
    Int_t fYDivider;
    Int_t fGasFile;
    Int_t fPadPlaneFile;
    Int_t fPadShapeFile;

    Double_t fBField;
    Double_t fTiltAng;
    Int_t fTB0;
    Double_t fThetaLorentz;
    Int_t fTBEntrance;
    Double_t fZPadPlane;
    Double_t fDensity;
    Double_t fThetaPad;
    Double_t fThetaRot;
    Double_t fGasPressure;
    Double_t fMaxRange;

    Double_t fEIonize;                 //!< effective ionization energy [eV]
    Double_t fCoefL;                   //!< longitudinal diffusion coefficient
    Double_t fCoefT;                   //!< transversal diffusion coefficient
    Double_t fGain;                    //!< gain factor from wire plane

  ClassDef(ATDigiPar, 1);
};

#endif
