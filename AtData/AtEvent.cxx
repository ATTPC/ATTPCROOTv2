#include "AtEvent.h"

#include "AtContainerManip.h"

#include <Rtypes.h>

#include <algorithm>
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
      return -1;
   } else
      return its->second;
}

void AtEvent::SortHitArray()
{
   std::sort(fHitArray.begin(), fHitArray.end(),
             [](const HitPtr &a, const HitPtr &b) { return AtHit::SortHit(*a, *b); });
}
void AtEvent::SortHitArrayID()
{
   std::sort(fHitArray.begin(), fHitArray.end(),
             [](const HitPtr &a, const HitPtr &b) { return a->GetHitID() < b->GetHitID(); });
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
