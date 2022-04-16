#include "AtProtoQuadrant.h"

#include <Rtypes.h>
#include <TH1.h>

#include <algorithm>
#include <cstddef>
#include <utility> // for move

ClassImp(AtProtoQuadrant);

AtProtoQuadrant::AtProtoQuadrant(Int_t QuadrantID) : AtProtoQuadrant({}, QuadrantID) {}

AtProtoQuadrant::AtProtoQuadrant(std::vector<AtHit> HitArray, Int_t QuadrantID)
   : AtProtoQuadrant(std::move(HitArray), QuadrantID, 0.0)
{
}

AtProtoQuadrant::AtProtoQuadrant(std::vector<AtHit> HitArray, Int_t QuadrantID, Double_t PhiQ)
   : fQuadrantID(QuadrantID), fHitArrayQ(std::move(HitArray)), fPhiQ(PhiQ)
{
}

void AtProtoQuadrant::SetEventID(Int_t evtid)
{
   fEventID = evtid;
}
void AtProtoQuadrant::AddHit(AtHit *hit)
{
   fHitArrayQ.push_back(*hit);
}
void AtProtoQuadrant::SetHitArray(std::vector<AtHit> *hitArray)
{
   fHitArrayQ = *hitArray;
}
void AtProtoQuadrant::SetQuadrantID(Int_t QuadrantID)
{
   fQuadrantID = QuadrantID;
}
void AtProtoQuadrant::SetPhiQ(Double_t PhiQ)
{
   fPhiQ = PhiQ;
}
void AtProtoQuadrant::SetPhiDistribution(TH1D *PhiD)
{
   fPhiDistr = *PhiD;
}
void AtProtoQuadrant::AddPhiVal(Double_t phival)
{
   fPhiDistrArray.push_back(phival);
}

Int_t AtProtoQuadrant::GetQuadrantID()
{
   return fQuadrantID;
}
Double_t AtProtoQuadrant::GetPhiQ()
{
   return fPhiQ;
}
Int_t AtProtoQuadrant::GetEventID()
{
   return fEventID;
}
std::size_t AtProtoQuadrant::GetNumHits()
{
   return fHitArrayQ.size();
}
TH1D *AtProtoQuadrant::GetPhiDistribution()
{
   return &fPhiDistr;
}
std::size_t AtProtoQuadrant::GetNumPhiVal()
{
   return fPhiDistrArray.size();
}

AtHit *AtProtoQuadrant::GetHit(Int_t hitNo)
{
   return (hitNo < GetNumHits() ? &fHitArrayQ[hitNo] : nullptr);
}

std::vector<AtHit> *AtProtoQuadrant::GetHitArray()
{
   return &fHitArrayQ;
}

std::vector<Double_t> *AtProtoQuadrant::GetPhiArray()
{
   return &fPhiDistrArray;
}
