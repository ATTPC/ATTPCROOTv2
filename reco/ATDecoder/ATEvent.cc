#include <iostream>
#include <iomanip>
#include <algorithm>

#include "ATEvent.hh"

ClassImp(ATEvent);

ATEvent::ATEvent(Bool_t isClustered, Bool_t isTracked, Bool_t isChanged)
:TNamed("ATEvent", "Event container")
{
  fEventID = -1;

  fIsClustered = isClustered;
  fIsTracked = isTracked;
  fIsChanged = isChanged;

  fIsGood = kFALSE;
  fQevent = -100.0;
  fRhoVariance = 0.0;

  
 
}

ATEvent::ATEvent(ATEvent *object)
:TNamed("ATEvent", "Event container")
{
  fEventID = object -> GetEventID();
  fQevent = -100.0;
  fRhoVariance = 0.0;
    
  fIsClustered = object -> IsClustered();
  fIsTracked = object -> IsTracked();
  fIsChanged = object -> IsChanged();

  fHitArray = *(object -> GetHitArray());

  if (IsClustered())
    //fClusterArray = *(object -> GetClusterArray());

  fIsGood = object -> IsGood();

 

}

ATEvent::~ATEvent()
{

}

void ATEvent::SetIsClustered(Bool_t value)   { fIsClustered = value; }
void ATEvent::SetIsTracked(Bool_t value)     { fIsTracked = value; }
void ATEvent::SetIsChanged(Bool_t value)     { fIsChanged = value; }

void ATEvent::SetIsGood(Bool_t value)        { fIsGood = value; }

Bool_t ATEvent::IsClustered()                { return fIsClustered; }
Bool_t ATEvent::IsTracked()                  { return fIsTracked; }
Bool_t ATEvent::IsChanged()                  { return fIsChanged; }

Bool_t ATEvent::IsGood()                     { return fIsGood; }

//Int_t ATEvent::GetNumClusters()              { return fClusterArray.size(); }

// setters
void ATEvent::SetEventID(Int_t evtid)                                 { fEventID = evtid; } 
void ATEvent::AddHit(ATHit *hit)                                      { fHitArray.push_back(*hit); } 
void ATEvent::SetHitArray(vector<ATHit> *hitArray)                    { fHitArray = *hitArray; }  
//void ATEvent::AddCluster(ATHitCluster *cluster)                   { fClusterArray.push_back(*cluster); } 
//void ATEvent::SetClusterArray(vector<ATHitCluster> *clusterArray) { fClusterArray = *clusterArray; }
void ATEvent::SetEventCharge(Double_t Qevent)  			      {fQevent = Qevent;}
void ATEvent::SetRhoVariance(Double_t RhoVariance)                    { fRhoVariance = RhoVariance;}
void ATEvent::SetMultiplicityMap(std::map<Int_t,Int_t> MultiMap)      { fMultiMap = MultiMap;}
void ATEvent::SetMeshSignal(Float_t *mesharray)			      { memcpy(fMeshSig, mesharray, sizeof(fMeshSig));}
void ATEvent::SetMeshSignal(Int_t idx, Float_t val)                   { fMeshSig[idx] = val; }

// getters
Int_t ATEvent::GetEventID() { return fEventID; }
Int_t ATEvent::GetNumHits() { return fHitArray.size(); }
Float_t *ATEvent::GetMesh()  { return fMeshSig; }

ATHit *ATEvent::GetHit(Int_t hitNo)
{
  return (hitNo < GetNumHits() ? &fHitArray[hitNo] : NULL);
}

void ATEvent::RemoveHit(Int_t hitNo)
{
  if (!(hitNo < GetNumHits()))
    return;

  fHitArray.erase(fHitArray.begin() + hitNo);
}

vector<ATHit> *ATEvent::GetHitArray()
{
  return &fHitArray;
}

/*ATHitCluster *ATEvent::GetCluster(Int_t clusterNo)
{
  if (!(clusterNo < GetNumClusters()) || !IsClustered())
    return NULL;

  return &fClusterArray[clusterNo];
}*/

/*void ATEvent::RemoveCluster(Int_t clusterNo)
{
  if (!(clusterNo < GetNumClusters()) || !IsClustered())
    return;

  fClusterArray.erase(fClusterArray.begin() + clusterNo);
}*/

/*vector<ATHitCluster> *ATEvent::GetClusterArray()
{
  return &fClusterArray;
}*/


Double_t ATEvent::GetEventCharge()  {return fQevent;}
Double_t ATEvent::GetRhoVariance()  { return fRhoVariance;}

Int_t ATEvent::GetHitPadMult(Int_t PadNum)
{

    std::map<Int_t,Int_t>::const_iterator its = fMultiMap.find(PadNum);
    Int_t padval = (*its).second;
    Int_t kIs = int(fMultiMap.find(PadNum) == fMultiMap.end());
    if(kIs){
                    std::cerr<<" = ATEvent::GetHitPadMult - PadNum not found "<<PadNum<<std::endl;
                    return -1;
    }else return padval;


}

Bool_t ATEvent::SortHitArray()
{
 
  std::sort(fHitArray.begin(),fHitArray.end(), SortHit);

}

Bool_t ATEvent::SortHitArrayTime()
{
 
  std::sort(fHitArray.begin(),fHitArray.end(), SortHitTime);

}

//Bool_t operator<(const ATHit &s1, const ATHit &s2){


//}















