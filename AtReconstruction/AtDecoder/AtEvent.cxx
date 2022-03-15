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
