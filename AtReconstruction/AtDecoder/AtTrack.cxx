#include "AtTrack.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(AtTrack)

AtTrack::AtTrack()
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

/*AtTrack::AtTrack(const AtTrack &obj)
{


}*/

AtTrack::~AtTrack()
{
}

void AtTrack::AddHit(AtHit *hit)                                    { fHitArray.push_back(*hit); }
void AtTrack::SetTrackID(Int_t val)                                 { fTrackID = val;}
void AtTrack::SetFitPar(std::vector<Double_t> par)                  { fParFit = par;}
void AtTrack::SetMinimum(Double_t min)                              { fMinimum = min;}
void AtTrack::SetNFree(Int_t ndf)                                   { fNFree = ndf;}
void AtTrack::SetAngleZAxis(Double_t angle)                         { fAngleZAxis = angle;}
void AtTrack::SetAngleZDet(Double_t angle)                          { fAngleZDet = angle;}
void AtTrack::SetAngleYDet(Double_t angle)                          { fAngleYDet = angle;}
void AtTrack::SetTrackVertex(TVector3 vertex)                       { fTrackVertex = vertex;}
void AtTrack::SetRange(Double_t range)                              { fRange = range;}
void AtTrack::SetGeoTheta(Double_t angle)                           { fGeoThetaAngle = angle;}
void AtTrack::SetGeoPhi(Double_t angle)                             { fGeoPhiAngle = angle;}
void AtTrack::SetGeoRange(Double_t range)                           { fRange = range;}
void AtTrack::SetQuadrant(Int_t quad)                               { fQuadrant = quad;}
void AtTrack::SetMCFit(Bool_t value)                                { kIsMCFit = value;}
void AtTrack::SetGeoEnergy(Double_t energy)                         { fGeoEnergy = energy;}
void AtTrack::SetGeoQEnergy(Double_t qenergy)                       { fGeoQEnergy = qenergy;}
void AtTrack::SetIsNoise(Bool_t value)                              { kIsNoise = value;}
void AtTrack::SetRANSACCoeff(std::vector<Double_t> par)             { fRANSACCoeff = par;}
void AtTrack::SetGeoCenter(std::pair<Double_t,Double_t> center)     { fGeoCenter = center; }
void AtTrack::SetGeoRadius(Double_t radius)                         { fGeoRadius = radius;}  

std::vector<AtHit> *AtTrack::GetHitArray()                          { return &fHitArray;}
std::vector<Double_t> AtTrack::GetFitPar()                          { return fParFit;}
Double_t AtTrack::GetMinimum()                                      { return fMinimum;}
Int_t AtTrack::GetNFree()                                           { return fNFree;}
Int_t AtTrack::GetTrackID()                                         { return fTrackID;}
Double_t AtTrack::GetAngleZAxis()                                   { return fAngleZAxis;}
Double_t AtTrack::GetAngleZDet()                                    { return fAngleZDet;}
Double_t AtTrack::GetAngleYDet()                                    { return fAngleYDet;}
TVector3 AtTrack::GetTrackVertex()                                  { return fTrackVertex;}
Int_t AtTrack::GetQuadrant()                                        { return fQuadrant;}
Double_t AtTrack::GetGeoTheta()                                     { return fGeoThetaAngle;}
Double_t AtTrack::GetGeoPhi()                                       { return fGeoPhiAngle;}
Double_t AtTrack::GetGeoEnergy()                                    { return fGeoEnergy;}
Bool_t  AtTrack::GetIsNoise()                                       { return kIsNoise;}
std::vector<Double_t>& AtTrack::GetRANSACCoeff()                    { return fRANSACCoeff;}
std::pair<Double_t,Double_t> AtTrack::GetGeoCenter()                { return fGeoCenter;}
Double_t                     AtTrack::GetGeoRadius()                { return fGeoRadius;}


std::vector<Double_t> AtTrack::GetPosXMin() const                   { return fPosXmin;}
std::vector<Double_t> AtTrack::GetPosYMin() const                   { return fPosYmin;}
std::vector<Double_t> AtTrack::GetPosZMin() const                   { return fPosZmin;}
std::vector<Double_t> AtTrack::GetPosXExp() const                   { return fPosXexp;}
std::vector<Double_t> AtTrack::GetPosYExp() const                   { return fPosYexp;}
std::vector<Double_t> AtTrack::GetPosZExp() const                   { return fPosZexp;}
std::vector<Double_t> AtTrack::GetPosXInt() const                   { return fPosXinter;}
std::vector<Double_t> AtTrack::GetPosYInt() const                   { return fPosYinter;}
std::vector<Double_t> AtTrack::GetPosZInt() const                   { return fPosZinter;}
std::vector<Double_t> AtTrack::GetPosXBack() const                  { return fPosXBack;}
std::vector<Double_t> AtTrack::GetPosYBack() const                  { return fPosYBack;}
std::vector<Double_t> AtTrack::GetPosZBack() const                  { return fPosZBack;}


TVector3 AtTrack::GetLastPoint()
{ 
	Double_t maxR = 0.; 
        TVector3 maxPos,temp; 
        for(Int_t nHit = 0;nHit < fHitArray.size();nHit++){
        	temp = fHitArray.at(nHit).GetPosition();
               	if(sqrt(pow(temp.X(),2) + pow(temp.Y(),2))>maxR){
                	maxR = sqrt(pow(temp.X(),2) + pow(temp.Y(),2));
                        maxPos = temp;
                }
        }
	return maxPos;
}
//alternative, but noticed that the last point in time is not alway the further away
/*
TVector3 AtTrack::GetLastPoint()
{
	TVector3 maxPos;
	if(fHitArray.size()>0){
		AtHit fhit = fHitArray.front(); // Last hit of the track (Low TB)
    		AtHit lhit = fHitArray.back(); // First hit of the track (High TB)
    		TVector3 fhitPos = fhit.GetPosition();
    		TVector3 lhitPos = lhit.GetPosition();
    		if( pow(fhitPos.X(),2) + pow(fhitPos.Y(),2) > pow(lhitPos.X(),2) + pow(lhitPos.Y(),2) ) maxPos = fhitPos;
		else maxPos = lhitPos;
	}
	return maxPos
}
*/


std::pair<Double_t,Double_t> AtTrack::GetThetaPhi(const TVector3 &vertex, const TVector3 &maxPos, int zdir)//zdir -1 for simu // +1 for data
{
	std::pair<Double_t,Double_t> thetaPhi;
        if(fParFit.size()>0){

                TVector3 vp(TMath::Sign(1,maxPos.X())*fabs(fParFit[1]),TMath::Sign(1,maxPos.Y())*fabs(fParFit[3]),zdir*TMath::Sign(1,(maxPos.Z()-vertex.Z()))*fabs(fParFit[5]));//works with simu
		//TVector3 vp(TMath::Sign(1,maxPos.X())*fabs(fParFit[1]),TMath::Sign(1,maxPos.Y())*fabs(fParFit[3]),TMath::Sign(1,(maxPos.Z()-vertex.Z()))*fabs(fParFit[5]));//works with data
//		std::cout<<" fParFit "<<fParFit[1]<<" "<<fParFit[3]<<" "<<fParFit[5]<<std::endl;
//		std::cout<<" maxPos "<<maxPos.X()<<" "<<maxPos.Y()<<" "<<maxPos.Z()<<std::endl;

		thetaPhi.first = vp.Theta();
		thetaPhi.second = vp.Phi();
	}
	return thetaPhi;
}


Double_t AtTrack::GetMeanTime()
{
    Double_t meanTime=0.0;

    if(fHitArray.size()>0)
    {
      Int_t sum = std::accumulate (begin(fHitArray), end(fHitArray),0,[](int i,AtHit& hit){return hit.GetTimeStamp() + i;});
      return meanTime = sum/(Double_t)fHitArray.size();
    }else return meanTime;

}

Double_t AtTrack::GetLinearRange()
{
  if(fHitArray.size()>0){
    AtHit fhit = fHitArray.front(); // Last hit of the track (Low TB)
    AtHit lhit = fHitArray.back(); // First hit of the track (High TB)
    TVector3 fhitPos = fhit.GetPosition();
    TVector3 lhitPos = lhit.GetPosition();

    return TMath::Sqrt( TMath::Power((fhitPos.X()-lhitPos.X()),2) + TMath::Power((fhitPos.Y()-lhitPos.Y()),2) + TMath::Power((fhitPos.Z()-lhitPos.Z()),2) );
  }else return 0;


}

Double_t AtTrack::GetLinearRange(TVector3 vertex)
{

  if(fHitArray.size()>0){
    AtHit fhit = fHitArray.front();
    TVector3 fhitPos = fhit.GetPosition();

    return TMath::Sqrt( TMath::Power((fhitPos.X()-vertex.X()),2) + TMath::Power((fhitPos.Y()-vertex.Y()),2) + TMath::Power((fhitPos.Z()-vertex.Z()),2) );
  }else return 0;


}


Double_t AtTrack::GetLinearRange(const TVector3 &vertex, const TVector3 &maxPos)
{
  if(fHitArray.size()>0){
    return TMath::Sqrt( TMath::Power((maxPos.X()-vertex.X()),2) + TMath::Power((maxPos.Y()-vertex.Y()),2) + TMath::Power((maxPos.Z()-vertex.Z()),2) );
  }else return 0;
}


Double_t AtTrack::GetGeoQEnergy()
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


void AtTrack::SetPosMin(const std::vector<Double_t> &xmin,const std::vector<Double_t> &ymin,const std::vector<Double_t> &zmin,const std::vector<Double_t> &xback,
  const std::vector<Double_t> &yback,const std::vector<Double_t> &zback)
{
  fPosXmin  = xmin;
  fPosYmin  = ymin;
  fPosZmin  = zmin;
  fPosXBack = xback;
  fPosYBack = yback;
  fPosZBack = zback;

}

void AtTrack::SetPosExp(const std::vector<Double_t> &xexp,const std::vector<Double_t> &yexp,const std::vector<Double_t> &zexp,const std::vector<Double_t> &xint,
  const std::vector<Double_t> &yint,const std::vector<Double_t> &zint)
{
  fPosXexp    = xexp;
  fPosYexp    = yexp;
  fPosZexp    = zexp;
  fPosXinter  = xint;
  fPosYinter  = yint;
  fPosZinter  = zint;
}

Bool_t AtTrack::SortHitArrayTime()
{

  std::sort(fHitArray.begin(),fHitArray.end(), SortHitTime);

}
