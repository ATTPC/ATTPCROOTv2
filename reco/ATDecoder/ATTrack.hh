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
    std::vector<ATHit> *GetHitArray();

  protected:
    std::vector<ATHit> fHitArray;
    Int_t fTrackID;

    ClassDef(ATTrack, 1);


};

#endif
