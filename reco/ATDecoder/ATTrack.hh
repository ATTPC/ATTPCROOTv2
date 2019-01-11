#ifndef ATTRACK_H
#define ATTRACK_H

#include "TROOT.h"
#include "TObject.h"
#include "TVector3.h"
#include "TMath.h"

#include <numeric>
#include <algorithm>
#include <iostream>

//ATTPCROOT
#include "ATHit.hh"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

class ATTrack : public TObject {

  public:
    ATTrack();
    //ATTrack(const ATTrack &obj);
    ~ATTrack();

    void AddHit(ATHit* hit);
    void SetTrackID(Int_t val);
    void SetFitPar(std::vector<Double_t> par);
    void SetMinimum(Double_t min); // Minimizer result
    void SetNFree(Int_t ndf);
    void SetAngleZAxis(Double_t angle);
    void SetAngleZDet(Double_t angle);
    void SetAngleYDet(Double_t angle);
    void SetTrackVertex(TVector3 vertex);
    void SetRange(Double_t range);
    void SetGeoTheta(Double_t angle);
    void SetGeoPhi(Double_t angle);
    void SetGeoRange(Double_t range);
    void SetQuadrant(Int_t quad);
    void SetMCFit(Bool_t value);
    void SetGeoEnergy(Double_t energy);
    void SetGeoQEnergy(Double_t qenergy);
    void SetIsNoise(Bool_t value);
    void SetRANSACCoeff(std::vector<Double_t> par);
    void SetGeoCenter(std::pair<Double_t,Double_t> center);
    void SetGeoRadius(Double_t radius);

    std::vector<ATHit>*          GetHitArray();
    std::vector<Double_t>        GetFitPar();
    Double_t                     GetMinimum();
    Int_t                        GetNFree();
    Int_t                        GetTrackID();
    Double_t                     GetAngleZAxis();
    Double_t                     GetAngleZDet();
    Double_t                     GetAngleYDet();
    Double_t                     GetMeanTime();
    Double_t                     GetLinearRange();
    Double_t                     GetLinearRange(TVector3 vertex);
    TVector3                     GetTrackVertex();
    Int_t                        GetQuadrant();
    Double_t                     GetGeoTheta();
    Double_t                     GetGeoPhi();
    Double_t                     GetGeoEnergy();
    Double_t                     GetGeoQEnergy();
    Bool_t                       GetIsNoise();
    std::vector<Double_t>&       GetRANSACCoeff();
    std::pair<Double_t,Double_t> GetGeoCenter();
    Double_t                     GetGeoRadius();
    Bool_t                       SortHitArrayTime();

    // MC result and projections
    std::vector<Double_t> fPosXmin;
    std::vector<Double_t> fPosYmin;
    std::vector<Double_t> fPosZmin;
    std::vector<Double_t> fPosXexp;
    std::vector<Double_t> fPosYexp;
    std::vector<Double_t> fPosZexp;
    std::vector<Double_t> fPosXinter;
    std::vector<Double_t> fPosYinter;
    std::vector<Double_t> fPosZinter;
    std::vector<Double_t> fPosXBack;
    std::vector<Double_t> fPosYBack;
    std::vector<Double_t> fPosZBack;

    std::vector<Double_t> GetPosXMin() const;
    std::vector<Double_t> GetPosYMin() const;
    std::vector<Double_t> GetPosZMin() const;
    std::vector<Double_t> GetPosXExp() const;
    std::vector<Double_t> GetPosYExp() const;
    std::vector<Double_t> GetPosZExp() const;
    std::vector<Double_t> GetPosXInt() const;
    std::vector<Double_t> GetPosYInt() const;
    std::vector<Double_t> GetPosZInt() const;
    std::vector<Double_t> GetPosXBack() const;
    std::vector<Double_t> GetPosYBack() const;
    std::vector<Double_t> GetPosZBack() const;

    void SetPosMin(const std::vector<Double_t> &xmin,const std::vector<Double_t> &ymin,const std::vector<Double_t> &zmin,const std::vector<Double_t> &xback,
      const std::vector<Double_t> &yback,const std::vector<Double_t> &zback);
    void SetPosExp(const std::vector<Double_t> &xexp,const std::vector<Double_t> &yexp,const std::vector<Double_t> &zexp,const std::vector<Double_t> &xint,
      const std::vector<Double_t> &yint,const std::vector<Double_t> &zint);

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
        Double_t sChi2Q;
        Double_t sChi2Range;
      };

      FitPar FitParameters;


  protected:
    std::vector<ATHit>           fHitArray;
    Int_t                        fTrackID;
    std::vector<Double_t>        fParFit;
    Double_t                     fMinimum; //Minimizer result
    Int_t                        fNFree; // Free paramets
    Double_t                     fAngleZAxis; // Angle of the track with respecto to the X axis.
    Double_t                     fAngleZDet; // Angle with respect to Z axis (beam axis) in the detector system.
    Double_t                     fAngleYDet;//  "         "           Y   "             "
    Double_t                     fRange; //Range of the particle
    TVector3                     fTrackVertex; //Mean Vertex of the track
    Double_t                     fGeoThetaAngle; // Geometrical scattering angle with respect to the detector FitParameters
    Double_t                     fGeoPhiAngle; //  " azimuthal "
    Double_t                     fGeoEnergy; //Energy from the range
    Double_t                     fGeoQEnergy; //Energy from the induced charge
    Int_t                        fQuadrant; //Pad plane quadrant
    std::vector<Double_t>        fRANSACCoeff; //Coefficients for radius smoothing using RANSAC: x, y and radius of curvature
    Double_t                     fGeoRadius; //Initial radius of curvature
    std::pair<Double_t,Double_t> fGeoCenter; //Center of the spiral track

    Bool_t kIsMCFit;
    Bool_t kIsNoise;

    static Bool_t SortHitTime(const ATHit &lhs, const ATHit &rhs)  { return lhs.fTimeStamp < rhs.fTimeStamp; }

    friend inline std::ostream& operator <<(std::ostream &o, const ATTrack &track)
    {
      std::cout<<cYELLOW<<" ====================================================== "<<std::endl;
      std::cout<<"  Track "<<track.fTrackID<<" Info : "<<std::endl;
      std::cout<<" Quadrant : "<<track.fQuadrant<<std::endl;
      std::cout<<" Geomterical Scattering Angle : "<<track.fGeoThetaAngle*(180.0/TMath::Pi())<<" deg "
      <<" - Geomterical Azimuthal Angle : "<<track.fGeoPhiAngle*(180.0/TMath::Pi())<<" deg "<<std::endl;
      std::cout<<" Geometrical Range : "<<track.fRange<<" mm "<<std::endl;
      std::cout<<" Angle with respect to Z axis : "<<track.fAngleZAxis<<cNORMAL<<std::endl;

      std::cout<<cRED<<" MC Fit : "<<track.kIsMCFit<<std::endl;
      std::cout<<" Number of simulated points : "<<track.GetPosXMin().size()<<std::endl;
      std::cout<<" Scattering Angle : "<<track.FitParameters.sThetaMin*(180.0/TMath::Pi())<<" deg "
      <<" - Azimuthal Angle : "<<track.FitParameters.sPhiMin*(180.0/TMath::Pi())<<" deg "<<std::endl;
      std::cout<<" Brho/Range : "<<track.FitParameters.sBrhoMin<<" T*m/mm "<<std::endl;
      std::cout<<" Energy :  "<<track.FitParameters.sEnerMin<<" MeV "<<std::endl;
      std::cout<<" Chi2 : "<<track.FitParameters.sChi2Min<<" - Chi2Q : "<<track.FitParameters.sChi2Q<<" - Chi2Range : "<<track.FitParameters.sChi2Range<<std::endl;
      std::cout<<cNORMAL<<std::endl;
      return o;
    }

    ClassDef(ATTrack, 2);


};

#endif
