#ifndef ATTRACK_H
#define ATTRACK_H

#include "TROOT.h"
#include "TObject.h"
#include "TVector3.h"

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

    std::vector<ATHit> *GetHitArray();
    std::vector<Double_t> GetFitPar();
    Double_t GetMinimum();
    Int_t GetNFree();
    Int_t GetTrackID();


  protected:
    std::vector<ATHit> fHitArray;
    Int_t fTrackID;
    std::vector<Double_t> fParFit;
    Double_t fMinimum; //Minimizer result
    Int_t fNFree; // Free paramets

    ClassDef(ATTrack, 1);


};

#endif
