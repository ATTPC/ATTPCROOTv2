#ifndef AtEVENT_H
#define AtEVENT_H

#include "TROOT.h"
#include "TObject.h"

#include <map>
#include <vector>

#include "AtHit.h"
#include "AtPad.h"



using std::vector;
using std::map;

class AtEvent : public TNamed {
public:

  // Constructor
  AtEvent(Bool_t isClustered = kFALSE, Bool_t isTracked = kFALSE, Bool_t isChanged = kFALSE);
  // Deep copy of event
  AtEvent(AtEvent *object);


  ~AtEvent();

  // setters
  void SetEventID(Int_t evtid);
  void SetTimestamp(ULong_t timestamp);
  void AddHit(AtHit *hit);
  void SetHitArray(vector<AtHit> *hitArray);

  void AddAuxPad(AtPad *pad);
  void SetAuxPadArray(vector<AtPad> *padArray);

  void SetEventCharge(Double_t Qevent);
  void SetRhoVariance(Double_t RhoVariance);


  void SetIsClustered(Bool_t value);
  void SetIsTracked(Bool_t value);
  void SetIsChanged(Bool_t value);
  void SetMultiplicityMap(std::map<Int_t,Int_t> MultiMap);
  void SetMeshSignal(Float_t *mesharray);
  void SetMeshSignal(Int_t idx, Float_t val);

  void SetIsGood(Bool_t value);
  void SetIsExtGate(Bool_t value);


  // getters
  Int_t GetEventID();
  Long_t GetTimestamp();
  Int_t GetNumHits();
  AtHit *GetHit(Int_t hitNo);
  void RemoveHit(Int_t hitNo);
  vector<AtHit> *GetHitArray();
  vector<AtHit>  GetHitArrayObj();

  std::vector<AtPad>* GetAuxPadArray();

  Bool_t IsExtGate();

  Int_t GetNumClusters();
  // AtHitCluster *GetCluster(Int_t clusterNo);
  // void RemoveCluster(Int_t clusterNo);
  // vector<AtHitCluster> *GetClusterArray();
  //
  // Int_t GetNumTracks();
  // STTrack *GetTrack(Int_t trackNo);
  // STTrack *RemoveTrack(Int_t trackNo);
  // vector<STTrack> *GetTrackArray();


  Double_t GetEventCharge();
  Double_t GetRhoVariance();
  Int_t    GetHitPadMult(Int_t PadNum); // Returns the multiplicity of the pad where this hit belongs to
  Float_t *GetMesh();

  Bool_t IsClustered();
  Bool_t IsTracked();
  Bool_t IsChanged();

  Bool_t IsGood();


  static Bool_t SortHit(const AtHit &lhs, const AtHit &rhs)  { return lhs.fPadNum < rhs.fPadNum; }
  static Bool_t SortHitTime(const AtHit &lhs, const AtHit &rhs)  { return lhs.fTimeStamp < rhs.fTimeStamp; }
  Bool_t SortHitArray();
  Bool_t SortHitArrayTime();

private:

  Bool_t fIsClustered;
  Bool_t fIsTracked;
  Bool_t fIsChanged;

  Bool_t fIsGood;
  Bool_t fIsinGate;

  Int_t fEventID;
  ULong_t fTimestamp;

  vector<AtHit> fHitArray;
  vector<AtPad> fAuxPadArray;

  Double_t fQevent;
  Double_t fRhoVariance;

  map<Int_t,Int_t> fMultiMap;

  Float_t fMeshSig[512];



  ClassDef(AtEvent, 3);
};

//Bool_t operator<(const AtHit &s1, const AtHit &s2);



#endif
