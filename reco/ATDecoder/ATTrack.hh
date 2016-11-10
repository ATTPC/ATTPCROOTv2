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
