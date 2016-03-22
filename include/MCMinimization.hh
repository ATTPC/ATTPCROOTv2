/*******************************************************************
* Stand alone class for Monte Carlo Minimization                   *
* Log: Class started 21-03-2016                                    *
* Author: Y. Ayyad and W. Mittig (NSCL)                            *
********************************************************************/
#ifndef MCMINIMIZATION_H
#define MCMINIMIZATION_H

#include "ATMinimization.hh"
#include "ATHit.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"
#include "TVector3.h"

class MCMinimization{

  public:
    MCMinimization();
    ~MCMinimization();

    Bool_t MinimizeOpt(Double_t* parameter,ATEvent *event);
    std::vector<ATHit> GetTBHitArray(Int_t TB,std::vector<ATHit> *harray);
    void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E);
    TVector3 TransformIniPos(Double_t x,Double_t y, Double_t z); //Transforms initial position from Pad plane to Lab frame
    TVector3 InvTransIniPos(Double_t x,Double_t y, Double_t z); //Transforms lab frame to pad plane
    void BackwardExtrapolation();
    void ResetParameters();

    struct FitPar
    {
      Double_t sThetaMin;
      Double_t sEnerMin;
      TVector3 sPosMin;
      Double_t sBrhoMin;
      Double_t sBMin;
      Double_t sPhiMin;
      Double_t sChi2Min;
      TVector3 sVertexPos;
      Double_t sVertexEner;
      Double_t sMinDistAppr;
      Int_t    sNumMCPoint;
      Double_t sNormChi2;

    };

    FitPar FitParameters;

    Double_t fThetaLorentz;
    Double_t fThetaRot;
    Double_t fThetaTilt;
    Double_t fThetaPad;
    Int_t    fEntTB;
    Double_t fZk;

    Double_t fThetaMin;
    Double_t fEnerMin;
    TVector3 fPosMin;
    Double_t fBrhoMin;
    Double_t fBMin;
    Double_t fPhiMin;
    Double_t fDensMin;
    Double_t fVertexEner;
    TVector3 fVertexPos;

    std::vector<Double_t> fPosXmin;
    std::vector<Double_t> fPosYmin;
    std::vector<Double_t> fPosZmin;
    std::vector<Int_t>    fPosTBmin;
    std::vector<Double_t> fPosXexp;
    std::vector<Double_t> fPosYexp;
    std::vector<Double_t> fPosZexp;
    std::vector<Double_t> fPosXinter;
    std::vector<Double_t> fPosYinter;
    std::vector<Double_t> fPosZinter;
    std::vector<Int_t>    fPosTBinter;

    std::vector<Double_t> fPosXBack;
    std::vector<Double_t> fPosYBack;
    std::vector<Double_t> fPosZBack;

    std::vector<Double_t> fPosTBexp;

    //Global variables
    Double_t sm1;
    Double_t m;
    Double_t dzstep;
    Int_t    integrationsteps;
    Double_t restmass;
    Double_t esm;
    Double_t iz1;
    Double_t z1;
    Double_t B0;
    Double_t B;

    Bool_t kVerbose;
    Bool_t kDebug;




};

#endif
