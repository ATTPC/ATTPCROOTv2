#include <iostream>
#include <iomanip>
#include <algorithm>

#include "AtEvent.h"

ClassImp(AtEvent);

AtEvent::AtEvent(Bool_t isClustered, Bool_t isTracked, Bool_t isChanged) : TNamed("AtEvent", "Event container")
{
   fEventID = -1;

   fIsClustered = isClustered;
   fIsTracked = isTracked;
   fIsChanged = isChanged;

   fIsGood = kFALSE;
   fQevent = -100.0;
   fRhoVariance = 0.0;
   fIsinGate = kFALSE;
}

AtEvent::AtEvent(AtEvent *object) : TNamed("AtEvent", "Event container")
{
   fEventID = object->GetEventID();
   fQevent = -100.0;
   fRhoVariance = 0.0;

   fIsClustered = object->IsClustered();
   fIsTracked = object->IsTracked();
   fIsChanged = object->IsChanged();

   fHitArray = *(object->GetHitArray());

   if (IsClustered())
      // fClusterArray = *(object -> GetClusterArray());

      fIsGood = object->IsGood();
}

AtEvent::~AtEvent() {}

void AtEvent::SetIsClustered(Bool_t value)
{
   fIsClustered = value;
}
void AtEvent::SetIsTracked(Bool_t value)
{
   fIsTracked = value;
}
void AtEvent::SetIsChanged(Bool_t value)
{
   fIsChanged = value;
}

void AtEvent::SetIsGood(Bool_t value)
{
   fIsGood = value;
}
void AtEvent::SetIsExtGate(Bool_t value)
{
   fIsinGate = value;
}

Bool_t AtEvent::IsClustered()
{
   return fIsClustered;
}
Bool_t AtEvent::IsTracked()
{
   return fIsTracked;
}
Bool_t AtEvent::IsChanged()
{
   return fIsChanged;
}

Bool_t AtEvent::IsGood()
{
   return fIsGood;
}
Bool_t AtEvent::IsExtGate()
{
   return fIsinGate;
}

// setters
void AtEvent::SetEventID(Int_t evtid)
{
   fEventID = evtid;
}
void AtEvent::SetTimestamp(ULong_t timestamp)
{
   fTimestamp = timestamp;
}
void AtEvent::AddHit(AtHit *hit)
{
   fHitArray.push_back(*hit);
}
void AtEvent::AddAuxPad(AtPad *pad)
{
   fAuxPadArray.push_back(*pad);
}
void AtEvent::SetHitArray(vector<AtHit> *hitArray)
{
   fHitArray = *hitArray;
}
void AtEvent::SetAuxPadArray(std::vector<AtPad> *padArray)
{
   fAuxPadArray = *padArray;
}
void AtEvent::SetEventCharge(Double_t Qevent)
{
   fQevent = Qevent;
}
void AtEvent::SetRhoVariance(Double_t RhoVariance)
{
   fRhoVariance = RhoVariance;
}
void AtEvent::SetMultiplicityMap(std::map<Int_t, Int_t> MultiMap)
{
   fMultiMap = MultiMap;
}
void AtEvent::SetMeshSignal(Float_t *mesharray)
{
   memcpy(fMeshSig, mesharray, sizeof(fMeshSig));
}
void AtEvent::SetMeshSignal(Int_t idx, Float_t val)
{
   fMeshSig[idx] = val;
}

// getters
Int_t AtEvent::GetEventID()
{
   return fEventID;
}
Long_t AtEvent::GetTimestamp()
{
   return fTimestamp;
}
Int_t AtEvent::GetNumHits()
{
   return fHitArray.size();
}
Float_t *AtEvent::GetMesh()
{
   return fMeshSig;
}

AtHit *AtEvent::GetHit(Int_t hitNo)
{
   return (hitNo < GetNumHits() ? &fHitArray[hitNo] : NULL);
}

void AtEvent::RemoveHit(Int_t hitNo)
{
   if (!(hitNo < GetNumHits()))
      return;

   fHitArray.erase(fHitArray.begin() + hitNo);
}

vector<AtHit> *AtEvent::GetHitArray()
{
   return &fHitArray;
}

vector<AtHit> AtEvent::GetHitArrayObj()
{
   return fHitArray;
}

std::vector<AtPad> *AtEvent::GetAuxPadArray()
{
   return &fAuxPadArray;
}

Double_t AtEvent::GetEventCharge()
{
   return fQevent;
}
Double_t AtEvent::GetRhoVariance()
{
   return fRhoVariance;
}

Int_t AtEvent::GetHitPadMult(Int_t PadNum)
{

   std::map<Int_t, Int_t>::const_iterator its = fMultiMap.find(PadNum);
   Int_t padval = (*its).second;
   Int_t kIs = int(fMultiMap.find(PadNum) == fMultiMap.end());
   if (kIs) {
      std::cerr << " = AtEvent::GetHitPadMult - PadNum not found " << PadNum << std::endl;
      return -1;
   } else
      return padval;
}

Bool_t AtEvent::SortHitArray()
{

   std::sort(fHitArray.begin(), fHitArray.end(), SortHit);
}

Bool_t AtEvent::SortHitArrayTime()
{

   std::sort(fHitArray.begin(), fHitArray.end(), SortHitTime);
}

// Bool_t operator<(const AtHit &s1, const AtHit &s2){

//}
