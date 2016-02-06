#include "ATProtoQuadrant.hh"

ClassImp(ATProtoQuadrant)

ATProtoQuadrant::ATProtoQuadrant()
{

   fEventID = -1;
   fPhiQ = 0.0;
   
}

ATProtoQuadrant::ATProtoQuadrant(Int_t QuadrantID)
{
  
    fEventID = -1;
    fQuadrantID = QuadrantID;
    fPhiQ = 0.0;
    
}


ATProtoQuadrant::ATProtoQuadrant(std::vector<ATHit> *HitArray,Int_t QuadrantID)
{
    fEventID = -1;
    fQuadrantID = QuadrantID;
    fHitArrayQ = *HitArray;
    fPhiQ = 0.0;
    
}

ATProtoQuadrant::ATProtoQuadrant(std::vector<ATHit> *HitArray,Int_t QuadrantID, Double_t PhiQ)
{
    fEventID = -1;
    fQuadrantID = QuadrantID;
    fHitArrayQ = *HitArray;
    fPhiQ = PhiQ;
    
}

ATProtoQuadrant::~ATProtoQuadrant()
{}

void ATProtoQuadrant::SetEventID(Int_t evtid)                             { fEventID = evtid; } 
void ATProtoQuadrant::AddHit(ATHit *hit)                                  { fHitArrayQ.push_back(*hit); }
void ATProtoQuadrant::SetHitArray(std::vector<ATHit> *hitArray)           { fHitArrayQ = *hitArray; }
void ATProtoQuadrant::SetQuadrantID(Int_t QuadrantID)                     { fQuadrantID = QuadrantID; }
void ATProtoQuadrant::SetPhiQ(Double_t PhiQ)                              { fPhiQ = PhiQ; }
void ATProtoQuadrant::SetPhiDistribution(TH1D* PhiD)                      { fPhiDistr = *PhiD;}
void ATProtoQuadrant::AddPhiVal(Double_t phival)                          { fPhiDistrArray.push_back(phival); }

Int_t ATProtoQuadrant::GetQuadrantID()      { return fQuadrantID; }
Double_t ATProtoQuadrant::GetPhiQ()         { return fPhiQ; }
Int_t ATProtoQuadrant::GetEventID()         { return fEventID; }
Int_t ATProtoQuadrant::GetNumHits()         { return fHitArrayQ.size(); }
TH1D* ATProtoQuadrant::GetPhiDistribution() { return &fPhiDistr;}
Int_t ATProtoQuadrant::GetNumPhiVal()       { return fPhiDistrArray.size(); }

ATHit *ATProtoQuadrant::GetHit(Int_t hitNo)
{
  return (hitNo < GetNumHits() ? &fHitArrayQ[hitNo] : NULL);
}


std::vector<ATHit> *ATProtoQuadrant::GetHitArray()
{
  return &fHitArrayQ;
}

std::vector<Double_t> *ATProtoQuadrant::GetPhiArray()
{
  return &fPhiDistrArray;
}

