/*******************************************************************
* Stand alone class for Monte Carlo Minimization                   *
* Log: Class started 08-08-2016                                    *
* Author: Y. Ayyad and W. Mittig (NSCL)                            *
********************************************************************/
#ifndef MCQMINIMIZATION_H
#define MCQMINIMIZATION_H

#include <boost/multi_array.hpp>

#include "ATMinimization.hh"
#include "ATHit.hh"
#include "ATEvent.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"
#include "TVector3.h"

//root
#include "TH2Poly.h"




class MCQMinimization{

  public:
    MCQMinimization();
    ~MCQMinimization();

    typedef boost::multi_array<double,3> multiarray;
    typedef multiarray::index index;

    Int_t GetMinimization();
    Bool_t MinimizeOptMapAmp(Double_t* parameter,ATEvent *event, TH2Poly* hPadPlane,multiarray PadCoord);

  protected:

    Int_t GetTBHit(Int_t TB,std::vector<ATHit> *harray);
    std::vector<ATHit> GetTBHitArray(Int_t TB,std::vector<ATHit> *harray);
    TVector3 TransformIniPos(Double_t x,Double_t y, Double_t z); //Transforms initial position from Pad plane to Lab frame
    TVector3 InvTransIniPos(Double_t x,Double_t y, Double_t z); //Transforms lab frame to pad plane
    void ResetParameters();

    void MCvar( double* parameter, int & modevar,int & iconvar,double & x0MC, double & y0MC, double & z0MC,
                double & aMC, double & phiMC, double & Bmin, double & dens, double & romin,
                double & x0MCv,  double & y0MCv, double & z0MCv, double & aMCv, double & phiMCv, double & Bminv,
                double & densv, double & rominv);

    void QMCsim(double* parameter, double* Qsim,double *zsimq,double & QMCtotal,
                double x0MC,double y0MC, double z0MC, double phiMCv,double aMCv,
                double Bminv, double densv, double rominv,double & e0sm, multiarray PadCoord);

    void Chi2MC(double  Qtrack[10000],double  ztrackq[10000],double & Qtracktotal,
                double  Qsim[10000],double  zsimq[10000],double & QMCtotal,
                double & Chi2fit, double & sigmaq, double & sigmaz);

    void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E);

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

    std::vector<ATHit> fHitTBArray;
   std::vector<ATHit> *fHitArray;



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

   Double_t fDriftVelocity;
   Int_t fTBTime;
   Double_t fBField;
   Double_t fTiltAng;
   Double_t fThetaTilt;
   Double_t fThetaPad;
   Double_t fThetaLorentz;
   Double_t fThetaRot;
   Double_t fZk;
   Int_t fEntTB; //Beam entrance Time Bucket

   TRotation* fPadtoDetRot;

   AtTpcMap *fMap;
   TH2Poly* fPadPlane;

   Bool_t kDebug;
   Bool_t kVerbose;

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



};

  #endif
