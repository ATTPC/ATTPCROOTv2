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

    std::vector<ATHit> *GetHitArray();
    std::vector<Double_t> GetFitPar();
    Double_t GetMinimum();
    Int_t    GetNFree();
    Int_t    GetTrackID();
    Double_t GetAngleZAxis();
    Double_t GetAngleZDet();
    Double_t GetAngleYDet();
    Double_t GetMeanTime();
    Double_t GetLinearRange();
    TVector3 GetTrackVertex();
    Int_t    GetQuadrant();
    Double_t GetGeoTheta();
    Double_t GetGeoPhi();


  protected:
    std::vector<ATHit> fHitArray;
    Int_t fTrackID;
    std::vector<Double_t> fParFit;
    Double_t fMinimum; //Minimizer result
    Int_t fNFree; // Free paramets
    Double_t fAngleZAxis; // Angle of the track with respecto to the X axis.
    Double_t fAngleZDet; // Angle with respect to Z axis (beam axis) in the detector system.
    Double_t fAngleYDet;//  "         "           Y   "             "
    Double_t fRange; //Range of the particle
    TVector3 fTrackVertex; //Mean Vertex of the track
    Double_t fGeoThetaAngle; // Geometrical scattering angle with respect to the detector FitParameters
    Double_t fGeoPhiAngle; //  " azimuthal "
    Int_t fQuadrant; //Pad plane quadrant

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

    void SetPosMin(std::vector<Double_t> xmin,std::vector<Double_t> ymin,std::vector<Double_t> zmin,std::vector<Double_t> xback,std::vector<Double_t> yback,std::vector<Double_t> zback);
    void SetPosExp(std::vector<Double_t> xexp,std::vector<Double_t> yexp,std::vector<Double_t> zexp,std::vector<Double_t> xint,std::vector<Double_t> yint,std::vector<Double_t> zint);

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


    friend inline std::ostream& operator <<(std::ostream &o, const ATTrack &track)
    {
      std::cout<<cYELLOW<<" ====================================================== "<<std::endl;
      std::cout<<"  Track "<<track.fTrackID<<" Info : "<<std::endl;
      std::cout<<" Quadrant : "<<track.fQuadrant<<std::endl;
      std::cout<<" Geomterical Scattering Angle : "<<track.fGeoThetaAngle*(180.0/TMath::Pi())<<" deg "
      <<" - Geomterical Anzimuthal Angle : "<<track.fGeoPhiAngle*(180.0/TMath::Pi())<<" deg "<<std::endl;
      std::cout<<" Geometrical Range : "<<track.fRange<<" mm "<<cNORMAL<<std::endl;
      return o;
    }

    ClassDef(ATTrack, 1);


};

#endif
