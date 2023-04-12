#include "AtEvent.h"

#include "AtContainerManip.h"
#include "AtRawEvent.h"

#include <Rtypes.h>

#include <algorithm>
#include <iostream>
#include <string> // for string

ClassImp(AtEvent);

AtEvent::AtEvent() : AtBaseEvent("AtEvent") {}

AtEvent::AtEvent(const AtEvent &copy)
   : AtBaseEvent(copy), fEventCharge(copy.fEventCharge), fRhoVariance(copy.fRhoVariance),
     fMultiplicityMap(copy.fMultiplicityMap), fMeshSig(copy.fMeshSig)
{
   for (const auto &hit : copy.fHitArray)
      fHitArray.push_back(hit->Clone());
}

// Here we are intentionally slicing to call the copy constructor to copy all the shared data between
// event types
AtEvent::AtEvent(const AtRawEvent &copy) : AtBaseEvent(copy) // NOLINT
{
   SetName("AtEvent");
}

AtEvent &AtEvent::operator=(AtEvent object)
{
   swap(*this, object);
   return *this;
}

void AtEvent::Clear(Option_t *opt)
{
   AtBaseEvent::Clear(opt);
   fEventCharge = -100;
   fRhoVariance = 0;
   fHitArray.clear();
   fMultiplicityMap.clear();
   fMeshSig.fill(0);
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
