#include "AtEvent.h"

#include "AtContainerManip.h"
#include "AtRawEvent.h"

#include <Rtypes.h>

#include <algorithm>
#include <iostream>

ClassImp(AtEvent);

AtEvent::AtEvent() : AtEvent(-1, false) {}

AtEvent::AtEvent(Int_t eventID, Bool_t isGood, Bool_t isInGate, ULong_t timestamp)
   : TNamed("AtEvent", "Event container"), fEventID(eventID), fIsGood(isGood), fIsInGate(isInGate),
     fTimestamp(timestamp)
{
}

AtEvent::AtEvent(const AtEvent &copy)
   : fEventID(copy.fEventID), fIsGood(copy.fIsGood), fIsInGate(copy.fIsInGate), fTimestamp(copy.fTimestamp),
     fEventCharge(copy.fEventCharge), fRhoVariance(copy.fRhoVariance), fAuxPadArray(copy.fAuxPadArray),
     fMultiplicityMap(copy.fMultiplicityMap), fMeshSig(copy.fMeshSig)
{
   for (const auto &hit : copy.fHitArray)
      fHitArray.push_back(hit->Clone());
}

AtEvent::AtEvent(const AtRawEvent &copy)
   : AtEvent(copy.GetEventID(), copy.IsGood(), copy.GetIsExtGate(), copy.GetTimestamp())
{
   for (const auto &[auxName, auxPad] : copy.GetAuxPads())
      fAuxPadArray.emplace_back(auxPad);
}
void AtEvent::Clear(Option_t *opt)
{
   fEventID = -1;
   fIsGood = false;
   fIsInGate = false;
   fEventCharge = -100;
   fRhoVariance = 0;
   fTimestamp = 0;
   fHitArray.clear();
   fAuxPadArray.clear();
   fMultiplicityMap.clear();
   fMeshSig.fill(0);
}

void AtEvent::CopyFrom(const AtEvent &inputEvent)
{
   this->fEventID = inputEvent.GetEventID();
   this->fIsGood = inputEvent.fIsGood;
   this->fIsInGate = inputEvent.fIsInGate;
   this->fEventCharge = inputEvent.fEventCharge;
   this->fRhoVariance = inputEvent.fRhoVariance;
   this->fTimestamp = inputEvent.fTimestamp;
   this->fAuxPadArray = inputEvent.fAuxPadArray;
   this->fMultiplicityMap = inputEvent.fMultiplicityMap;
   this->fMeshSig = inputEvent.fMeshSig;
}

void AtEvent::SetMeshSignal(Int_t idx, Float_t val)
{
   fMeshSig[idx] = val;
}

Int_t AtEvent::GetHitPadMult(Int_t PadNum)
{
   auto its = fMultiplicityMap.find(PadNum);
   if (its == fMultiplicityMap.end()) {
      std::cerr << " = AtEvent::GetHitPadMult - PadNum not found " << PadNum << std::endl;
      std::cout << fMultiplicityMap.size() << std::endl;
      for (const auto &pair : fMultiplicityMap)
         std::cout << "    " << pair.first << " " << pair.second << std::endl;
      return -1;
   } else
      return its->second;
}

void AtEvent::SortHitArray()
{
   std::sort(fHitArray.begin(), fHitArray.end(),
             [](const HitPtr &a, const HitPtr &b) { return AtHit::SortHit(*a, *b); });
}

void AtEvent::SortHitArrayTime()
{
   std::sort(fHitArray.begin(), fHitArray.end(),
             [](const HitPtr &a, const HitPtr &b) { return AtHit::SortHitTime(*a, *b); });
}

std::vector<AtHit> AtEvent::GetHitArray() const
{
   return ContainerManip::GetObjectVector(fHitArray);
}
