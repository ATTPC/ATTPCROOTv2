#include "ATTrack.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(ATTrack)

ATTrack::ATTrack()
{
  fTrackID     = -1;
  fMinimum     = -1;
  fNFree       = -1;
  fAngleZAxis  = -20;
  fAngleZDet   = -20;
  fAngleYDet  = -20;
  fRange = 0.0;
}

/*ATTrack::ATTrack(const ATTrack &obj)
{


}*/

ATTrack::~ATTrack()
{
}

void ATTrack::AddHit(ATHit *hit)                                    { fHitArray.push_back(*hit); }
void ATTrack::SetTrackID(Int_t val)                                 { fTrackID = val;}
void ATTrack::SetFitPar(std::vector<Double_t> par)                  { fParFit = par;}
void ATTrack::SetMinimum(Double_t min)                              { fMinimum = min;}
void ATTrack::SetNFree(Int_t ndf)                                   { fNFree = ndf;}
void ATTrack::SetAngleZAxis(Double_t angle)                         { fAngleZAxis = angle;}
void ATTrack::SetAngleZDet(Double_t angle)                          { fAngleZDet = angle;}
void ATTrack::SetAngleYDet(Double_t angle)                          { fAngleYDet = angle;}
void ATTrack::SetTrackVertex(TVector3 vertex)                       { fTrackVertex = vertex;}
void ATTrack::SetRange(Double_t range)                              { fRange = range;}
void ATTrack::SetGeoTheta(Double_t angle)                           { fGeoThetaAngle = angle;}
void ATTrack::SetGeoPhi(Double_t angle)                             { fGeoPhiAngle = angle;}

std::vector<ATHit> *ATTrack::GetHitArray()                          { return &fHitArray;}
std::vector<Double_t> ATTrack::GetFitPar()                          { return fParFit;}
Double_t ATTrack::GetMinimum()                                      { return fMinimum;}
Int_t ATTrack::GetNFree()                                           { return fNFree;}
Int_t ATTrack::GetTrackID()                                         { return fTrackID;}
Double_t ATTrack::GetAngleZAxis()                                   { return fAngleZAxis;}
Double_t ATTrack::GetAngleZDet()                                    { return fAngleZDet;}
Double_t ATTrack::GetAngleYDet()                                    { return fAngleYDet;}
TVector3 ATTrack::GetTrackVertex()                                  { return fTrackVertex;}


Double_t ATTrack::GetMeanTime()
{
    Double_t meanTime=0.0;

    if(fHitArray.size()>0)
    {
      Int_t sum = std::accumulate (begin(fHitArray), end(fHitArray),0,[](int i,ATHit& hit){return hit.GetTimeStamp() + i;});
      return meanTime = sum/(Double_t)fHitArray.size();
    }else return meanTime;

}

Double_t ATTrack::GetLinearRange()
{
  if(fHitArray.size()>0){
    ATHit fhit = fHitArray.front();
    ATHit lhit = fHitArray.back();
    TVector3 fhitPos = fhit.GetPosition();
    TVector3 lhitPos = lhit.GetPosition();
    return TMath::Sqrt( TMath::Power((fhitPos.X()-lhitPos.X()),2) + TMath::Power((fhitPos.Y()-lhitPos.Y()),2) + TMath::Power((fhitPos.Z()-lhitPos.Z()),2) );
  }else return 0;


}
