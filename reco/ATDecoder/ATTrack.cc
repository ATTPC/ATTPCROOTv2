#include "ATTrack.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(ATTrack)

ATTrack::ATTrack()
{
}

ATTrack::~ATTrack()
{
}

void ATTrack::AddHit(ATHit *hit)                                    { fHitArray.push_back(*hit); }
void ATTrack::SetTrackID(Int_t val)                                 { fTrackID = val;}
std::vector<ATHit> *ATTrack::GetHitArray()                          { return &fHitArray;}
