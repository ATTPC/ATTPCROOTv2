#include "AtProtoQuadrant.h"

#include <TH1.h>
#include <stddef.h>
#include <algorithm>

#include "Rtypes.h"

ClassImp(AtProtoQuadrant)

   AtProtoQuadrant::AtProtoQuadrant()
{

   fEventID = -1;
   fPhiQ = 0.0;
}

AtProtoQuadrant::AtProtoQuadrant(Int_t QuadrantID)
{

   fEventID = -1;
   fQuadrantID = QuadrantID;
   fPhiQ = 0.0;
}

AtProtoQuadrant::AtProtoQuadrant(std::vector<AtHit> *HitArray, Int_t QuadrantID)
{
   fEventID = -1;
   fQuadrantID = QuadrantID;
   fHitArrayQ = *HitArray;
   fPhiQ = 0.0;
}

AtProtoQuadrant::AtProtoQuadrant(std::vector<AtHit> *HitArray, Int_t QuadrantID, Double_t PhiQ)
{
   fEventID = -1;
   fQuadrantID = QuadrantID;
   fHitArrayQ = *HitArray;
   fPhiQ = PhiQ;
}

AtProtoQuadrant::~AtProtoQuadrant() {}

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
Int_t AtProtoQuadrant::GetNumHits()
{
   return fHitArrayQ.size();
}
TH1D *AtProtoQuadrant::GetPhiDistribution()
{
   return &fPhiDistr;
}
Int_t AtProtoQuadrant::GetNumPhiVal()
{
   return fPhiDistrArray.size();
}

AtHit *AtProtoQuadrant::GetHit(Int_t hitNo)
{
   return (hitNo < GetNumHits() ? &fHitArrayQ[hitNo] : NULL);
}

std::vector<AtHit> *AtProtoQuadrant::GetHitArray()
{
   return &fHitArrayQ;
}

std::vector<Double_t> *AtProtoQuadrant::GetPhiArray()
{
   return &fPhiDistrArray;
}
