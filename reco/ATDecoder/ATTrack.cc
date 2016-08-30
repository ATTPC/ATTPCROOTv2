#include "ATTrack.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(ATTrack)

ATTrack::ATTrack()
{
  fTrackID = -1;
  fMinimum = -1;
  fNFree   = -1;
}

ATTrack::~ATTrack()
{
}

void ATTrack::AddHit(ATHit *hit)                                    { fHitArray.push_back(*hit); }
void ATTrack::SetTrackID(Int_t val)                                 { fTrackID = val;}
void ATTrack::SetFitPar(std::vector<Double_t> par)                  { fParFit = par;}
void ATTrack::SetMinimum(Double_t min)                              { fMinimum = min;}
void ATTrack::SetNFree(Int_t ndf)                                   { fNFree = ndf;}

std::vector<ATHit> *ATTrack::GetHitArray()                          { return &fHitArray;}
std::vector<Double_t> ATTrack::GetFitPar()                          { return fParFit;}
Double_t ATTrack::GetMinimum()                                      { return fMinimum;}
Int_t ATTrack::GetNFree()                                           { return fNFree;}
