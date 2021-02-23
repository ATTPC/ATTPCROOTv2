/*******************************************************************
* Class for Monte Carlo Minimization                               *
* Log: Class started 28-10-2015                                    *
* Author: Y. Ayyad and W. Mittig (NSCL)                            *
********************************************************************/

#ifndef AtMCMINIMIZAtION_H
#define AtMCMINIMIZAtION_H

#include "AtMinimization.h"
#include "AtHit.h"
#include "AtDigiPar.h"

#include "AtTpcMap.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"
#include "TVector3.h"

class AtMCMinimization : public AtMinimization{

      public:
	       AtMCMinimization();
        ~AtMCMinimization();

        Int_t GetMinimization();
    	  Bool_t Minimize(Double_t* parameter,AtEvent *event);
        Bool_t MinimizeOpt(Double_t* parameter,AtEvent *event);
        Bool_t MinimizeOptMap(Double_t* parameter,AtEvent *event, TH2Poly* hPadPlane);
        Bool_t MinimizeOptMapAmp(Double_t* parameter,AtEvent *event, TH2Poly* hPadPlane,const multiarray& PadCoord);
        std::vector<Double_t> GetPosXMin();
        std::vector<Double_t> GetPosYMin();
        std::vector<Double_t> GetPosZMin();
        std::vector<Double_t> GetPosXExp();
        std::vector<Double_t> GetPosYExp();
        std::vector<Double_t> GetPosZExp();
        std::vector<Double_t> GetPosXInt();
        std::vector<Double_t> GetPosYInt();
        std::vector<Double_t> GetPosZInt();
        std::vector<Double_t> GetPosXBack();
        std::vector<Double_t> GetPosYBack();
        std::vector<Double_t> GetPosZBack();

        std::vector<AtHit> GetTBHitArray(Int_t TB,std::vector<AtHit> *harray);


      protected:
          void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E);
          Double_t GetSimThetaAngle(TVector3* pos, TVector3* posforw);
          void BackwardExtrapolation();
          void SetMap(AtTpcMap* map);

          Int_t GetTBHit(Int_t TB,std::vector<AtHit> *harray);
          TVector3 TransformIniPos(Double_t x,Double_t y, Double_t z); //Transforms initial position from Pad plane to Lab frame
          TVector3 InvTransIniPos(Double_t x,Double_t y, Double_t z); //Transforms lab frame to pad plane
          void ResetParameters();

          std::vector<AtHit> fHitTBArray;
          std::vector<AtHit> *fHitArray;

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
          Double_t fThetaPad;
          Double_t fThetaLorentz;
          Double_t fThetaRot;
          Double_t fZk;
          Int_t fEntTB; //Beam entrance Time Bucket

          TRotation* fPadtoDetRot;

          AtTpcMap *fMap;

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




        ClassDef(AtMCMinimization, 1);

};

#endif
