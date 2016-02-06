/*******************************************************************
* Class for Monte Carlo Minimization                               *
* Log: Class started 28-10-2015                                    *
* Author: Y. Ayyad and W. Mittig (NSCL)                            *
********************************************************************/

#ifndef ATMCMINIMIZATION_H
#define ATMCMINIMIZATION_H

#include "ATMinimization.hh"
#include "ATHit.hh"
#include "ATDigiPar.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"
#include "TVector3.h"

class ATMCMinimization : public ATMinimization{

      public:
	       ATMCMinimization();
        ~ATMCMinimization();

        Int_t GetMinimization();
    	  Bool_t Minimize(Double_t* parameter,ATEvent *event);
        std::vector<Double_t> GetPosXMin();
        std::vector<Double_t> GetPosYMin();
        std::vector<Double_t> GetPosZMin();
        std::vector<Double_t> GetPosXExp();
        std::vector<Double_t> GetPosYExp();
        std::vector<Double_t> GetPosZExp();
        std::vector<Double_t> GetPosXInt();
        std::vector<Double_t> GetPosYInt();
        std::vector<Double_t> GetPosZInt();

        std::vector<ATHit> GetTBHitArray(Int_t TB,std::vector<ATHit> *harray);


      protected:
          void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E);
          Double_t GetSimThetaAngle(TVector3* pos, TVector3* posforw);

          Int_t GetTBHit(Int_t TB,std::vector<ATHit> *harray);
          TVector3 TransformIniPos(Double_t x,Double_t y, Double_t z); //Transforms initial position from Pad plane to Lab frame
          TVector3 InvTransIniPos(Double_t x,Double_t y, Double_t z); //Transforms lab frame to pad plane

          std::vector<ATHit> fHitTBArray;
          std::vector<ATHit> *fHitArray;

          Double_t fThetaMin;
          Double_t fEnerMin;
          TVector3 fPosMin;
          Double_t fBrhoMin;
          Double_t fBMin;
          Double_t fPhiMin;
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

          std::vector<Double_t> fPosTBexp;

          Double_t fDriftVelocity;
          Int_t fTBTime;
          Double_t fBField;
          Double_t fTiltAng;
          Double_t fThetaPad;
          Double_t fThetaLorentz;
          Double_t fThetaRot;
          Double_t fZk;
          Int_t fEntTB; //Beam entrance Time Bucket

          TRotation* fPadtoDetRot;

          Bool_t kDebug;






        ClassDef(ATMCMinimization, 1);

};

#endif
