#include <iostream>
#include <iomanip>
#include <algorithm>
#include <iostream>

#include "AtEvent.h"

ClassImp(AtEvent);

AtEvent::AtEvent() : AtEvent(-1, false) {}

AtEvent::AtEvent(Int_t eventID, Bool_t isGood)
   : TNamed("AtEvent", "Event container"), fEventID(eventID), fIsGood(isGood)
{
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
void AtEvent::SetMeshSignal(const traceArray &mesharray)
{
   fMeshSig = mesharray;
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
   std::sort(fHitArray.begin(), fHitArray.end(), AtHit::SortHit);
}

void AtEvent::SortHitArrayTime()
{
   std::sort(fHitArray.begin(), fHitArray.end(), AtHit::SortHitTime);
}
