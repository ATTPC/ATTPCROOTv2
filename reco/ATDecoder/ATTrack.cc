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
  fAngleYDet   = -20;
  fRange       = 0.0;
  fQuadrant    = -1;
  fGeoEnergy   = -10.0;
  fGeoQEnergy  = -10.0;
  kIsMCFit     = kFALSE;
  kIsNoise     = kFALSE;
  FitParameters.sThetaMin        = 0;
  FitParameters.sThetaMin        = 0;
  FitParameters.sEnerMin         = 0;
  FitParameters.sPosMin.SetXYZ(0,0,0);
  FitParameters.sBrhoMin         = 0;
  FitParameters.sBMin            = 0;
  FitParameters.sPhiMin          = 0;
  FitParameters.sChi2Min         = 0;
  FitParameters.sVertexPos.SetXYZ(0,0,0);
  FitParameters.sVertexEner      = 0;
  FitParameters.sMinDistAppr     = 0;
  FitParameters.sNumMCPoint      = 0;
  FitParameters.sNormChi2        = 0;
  FitParameters.sChi2Q           = 0;
  FitParameters.sChi2Range       = 0;

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
void ATTrack::SetGeoRange(Double_t range)                           { fRange = range;}
void ATTrack::SetQuadrant(Int_t quad)                               { fQuadrant = quad;}
void ATTrack::SetMCFit(Bool_t value)                                { kIsMCFit = value;}
void ATTrack::SetGeoEnergy(Double_t energy)                         { fGeoEnergy = energy;}
void ATTrack::SetGeoQEnergy(Double_t qenergy)                       { fGeoQEnergy = qenergy;}
void ATTrack::SetIsNoise(Bool_t value)                              { kIsNoise = value;}

std::vector<ATHit> *ATTrack::GetHitArray()                          { return &fHitArray;}
std::vector<Double_t> ATTrack::GetFitPar()                          { return fParFit;}
Double_t ATTrack::GetMinimum()                                      { return fMinimum;}
Int_t ATTrack::GetNFree()                                           { return fNFree;}
Int_t ATTrack::GetTrackID()                                         { return fTrackID;}
Double_t ATTrack::GetAngleZAxis()                                   { return fAngleZAxis;}
Double_t ATTrack::GetAngleZDet()                                    { return fAngleZDet;}
Double_t ATTrack::GetAngleYDet()                                    { return fAngleYDet;}
TVector3 ATTrack::GetTrackVertex()                                  { return fTrackVertex;}
Int_t ATTrack::GetQuadrant()                                        { return fQuadrant;}
Double_t ATTrack::GetGeoTheta()                                     { return fGeoThetaAngle;}
Double_t ATTrack::GetGeoPhi()                                       { return fGeoPhiAngle;}
Double_t ATTrack::GetGeoEnergy()                                    { return fGeoEnergy;}
Bool_t  ATTrack::GetIsNoise()                                       { return kIsNoise;}

std::vector<Double_t> ATTrack::GetPosXMin() const                   { return fPosXmin;}
std::vector<Double_t> ATTrack::GetPosYMin() const                   { return fPosYmin;}
std::vector<Double_t> ATTrack::GetPosZMin() const                   { return fPosZmin;}
std::vector<Double_t> ATTrack::GetPosXExp() const                   { return fPosXexp;}
std::vector<Double_t> ATTrack::GetPosYExp() const                   { return fPosYexp;}
std::vector<Double_t> ATTrack::GetPosZExp() const                   { return fPosZexp;}
std::vector<Double_t> ATTrack::GetPosXInt() const                   { return fPosXinter;}
std::vector<Double_t> ATTrack::GetPosYInt() const                   { return fPosYinter;}
std::vector<Double_t> ATTrack::GetPosZInt() const                   { return fPosZinter;}
std::vector<Double_t> ATTrack::GetPosXBack() const                  { return fPosXBack;}
std::vector<Double_t> ATTrack::GetPosYBack() const                  { return fPosYBack;}
std::vector<Double_t> ATTrack::GetPosZBack() const                  { return fPosZBack;}



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
    ATHit fhit = fHitArray.front(); // Last hit of the track (Low TB)
    ATHit lhit = fHitArray.back(); // First hit of the track (High TB)
    TVector3 fhitPos = fhit.GetPosition();
    TVector3 lhitPos = lhit.GetPosition();

    return TMath::Sqrt( TMath::Power((fhitPos.X()-lhitPos.X()),2) + TMath::Power((fhitPos.Y()-lhitPos.Y()),2) + TMath::Power((fhitPos.Z()-lhitPos.Z()),2) );
  }else return 0;


}

Double_t ATTrack::GetLinearRange(TVector3 vertex)
{

  if(fHitArray.size()>0){
    ATHit fhit = fHitArray.front();
    TVector3 fhitPos = fhit.GetPosition();

    return TMath::Sqrt( TMath::Power((fhitPos.X()-vertex.X()),2) + TMath::Power((fhitPos.Y()-vertex.Y()),2) + TMath::Power((fhitPos.Z()-vertex.Z()),2) );
  }else return 0;


}

Double_t ATTrack::GetGeoQEnergy()
{

      Double_t charge = 0;

      if(fHitArray.size()>0){
          for(Int_t i=0;i<fHitArray.size();i++)
          {
            charge+= fHitArray.at(i).GetCharge();
          }

          return charge;

      }else return -10.0;

}


void ATTrack::SetPosMin(const std::vector<Double_t> &xmin,const std::vector<Double_t> &ymin,const std::vector<Double_t> &zmin,const std::vector<Double_t> &xback,
  const std::vector<Double_t> &yback,const std::vector<Double_t> &zback)
{
  fPosXmin  = xmin;
  fPosYmin  = ymin;
  fPosZmin  = zmin;
  fPosXBack = xback;
  fPosYBack = yback;
  fPosZBack = zback;

}

void ATTrack::SetPosExp(const std::vector<Double_t> &xexp,const std::vector<Double_t> &yexp,const std::vector<Double_t> &zexp,const std::vector<Double_t> &xint,
  const std::vector<Double_t> &yint,const std::vector<Double_t> &zint)
{
  fPosXexp    = xexp;
  fPosYexp    = yexp;
  fPosZexp    = zexp;
  fPosXinter  = xint;
  fPosYinter  = yint;
  fPosZinter  = zint;
}
