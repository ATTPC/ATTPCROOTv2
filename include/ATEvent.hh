#ifndef ATEVENT_H
#define ATEVENT_H

#include "TROOT.h"
#include "TObject.h"

#include <vector>

#include "ATHit.hh"
#include "ATPad.hh"
#include <map>


using std::vector;

class ATEvent : public TNamed {
  public:
    ATEvent(Bool_t isClustered = kFALSE, Bool_t isTracked = kFALSE, Bool_t isChanged = kFALSE);
    ATEvent(ATEvent *object);
    ~ATEvent();

    // setters
    void SetEventID(Int_t evtid);
    void AddHit(ATHit *hit);
    void SetHitArray(vector<ATHit> *hitArray);

    void AddAuxPad(ATPad *pad);
    void SetAuxPadArray(vector<ATPad> *padArray);

    void SetEventCharge(Double_t Qevent);
    void SetRhoVariance(Double_t RhoVariance);


    void SetIsClustered(Bool_t value);
    void SetIsTracked(Bool_t value);
    void SetIsChanged(Bool_t value);
    void SetMultiplicityMap(std::map<Int_t,Int_t> MultiMap);
    void SetMeshSignal(Float_t *mesharray);
    void SetMeshSignal(Int_t idx, Float_t val);

    void SetIsGood(Bool_t value);
    

    // getters
    Int_t GetEventID();

    Int_t GetNumHits();
    ATHit *GetHit(Int_t hitNo);
    void RemoveHit(Int_t hitNo);
    vector<ATHit> *GetHitArray();
    vector<ATHit>  GetHitArrayObj();

    std::vector<ATPad>* GetAuxPadArray();

    Int_t GetNumClusters();
    //ATHitCluster *GetCluster(Int_t clusterNo);
    //void RemoveCluster(Int_t clusterNo);
    //vector<ATHitCluster> *GetClusterArray();

//    Int_t GetNumTracks();
//    STTrack *GetTrack(Int_t trackNo);
//    STTrack *RemoveTrack(Int_t trackNo);
//    vector<STTrack> *GetTrackArray();


    Double_t GetEventCharge();
    Double_t GetRhoVariance();
    Int_t GetHitPadMult(Int_t PadNum); // Returns the multiplicity of the pad where this hit belongs to
    Float_t *GetMesh();

    Bool_t IsClustered();
    Bool_t IsTracked();
    Bool_t IsChanged();

    Bool_t IsGood();

    static Bool_t SortHit(const ATHit &lhs, const ATHit &rhs)  { return lhs.fPadNum < rhs.fPadNum; }
    static Bool_t SortHitTime(const ATHit &lhs, const ATHit &rhs)  { return lhs.fTimeStamp < rhs.fTimeStamp; }
    Bool_t SortHitArray();
    Bool_t SortHitArrayTime();

  private:
    Bool_t fIsClustered;
    Bool_t fIsTracked;
    Bool_t fIsChanged;

    Bool_t fIsGood;

    Int_t fEventID;

    vector<ATHit> fHitArray;
    std::vector<ATPad> fAuxPadArray;
    Double_t fQevent;
    Double_t fRhoVariance;
    std::map<Int_t,Int_t> fMultiMap;

    Float_t fMeshSig[512];



  ClassDef(ATEvent, 3);
};

//Bool_t operator<(const ATHit &s1, const ATHit &s2);



#endif
