#ifndef ATTRACK_H
#define ATTRACK_H

#include "TROOT.h"
#include "TObject.h"
#include "TVector3.h"
#include "TMath.h"

#include <numeric>
#include <algorithm>

//ATTPCROOT
#include "ATHit.hh"

class ATTrack : public TObject {

  public:
    ATTrack();
    ~ATTrack();

    void AddHit(ATHit* hit);
    void SetTrackID(Int_t val);
    void SetFitPar(std::vector<Double_t> par);
    void SetMinimum(Double_t min); // Minimizer result
    void SetNFree(Int_t ndf);
    void SetAngleZAxis(Double_t angle);
    void SetTrackVertex(TVector3 vertex);

    std::vector<ATHit> *GetHitArray();
    std::vector<Double_t> GetFitPar();
    Double_t GetMinimum();
    Int_t GetNFree();
    Int_t GetTrackID();
    Double_t GetAngleZAxis();
    Double_t GetMeanTime();
    Double_t GetLinearRange();
    TVector3 GetTrackVertex();


  protected:
    std::vector<ATHit> fHitArray;
    Int_t fTrackID;
    std::vector<Double_t> fParFit;
    Double_t fMinimum; //Minimizer result
    Int_t fNFree; // Free paramets
    Double_t fAngleZAxis; // Angle of the track with respecto to the X axis.
    Double_t fRange; //Range of the particle
    TVector3 fTrackVertex; //Mean Vertex of the track


    ClassDef(ATTrack, 1);


};

#endif
