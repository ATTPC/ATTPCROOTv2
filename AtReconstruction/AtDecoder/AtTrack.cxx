#include "AtTrack.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"
#include "Math/Point3D.h"

ClassImp(AtTrack);

AtTrack::AtTrack()
{
   fTrackID = -1;
   fMinimum = -1;
   fNFree = -1;
   fAngleZAxis = -20;
   fAngleZDet = -20;
   fAngleYDet = -20;
   kIsNoise = kFALSE;
}

void AtTrack::SetNFree(Int_t ndf)
{
   fNFree = ndf;
}
void AtTrack::SetAngleZAxis(Double_t angle)
{
   fAngleZAxis = angle;
}
void AtTrack::SetAngleZDet(Double_t angle)
{
   fAngleZDet = angle;
}
void AtTrack::SetAngleYDet(Double_t angle)
{
   fAngleYDet = angle;
}
void AtTrack::SetTrackVertex(XYZPoint vertex)
{
   fTrackVertex = vertex;
}
void AtTrack::SetGeoTheta(Double_t angle)
{
   fGeoThetaAngle = angle;
}
void AtTrack::SetGeoPhi(Double_t angle)
{
   fGeoPhiAngle = angle;
}
void AtTrack::SetIsNoise(Bool_t value)
{
   kIsNoise = value;
}
void AtTrack::SetRANSACCoeff(std::vector<Double_t> par)
{
   fRANSACCoeff = par;
}
void AtTrack::SetGeoCenter(std::pair<Double_t, Double_t> center)
{
   fGeoCenter = center;
}
void AtTrack::SetGeoRadius(Double_t radius)
{
   fGeoRadius = radius;
}

std::vector<AtHitCluster> *AtTrack::GetHitClusterArray()
{
   return &fHitClusterArray;
}
std::vector<Double_t> AtTrack::GetFitPar()
{
   return fFitPar;
}
Double_t AtTrack::GetMinimum()
{
   return fMinimum;
}
Int_t AtTrack::GetNFree()
{
   return fNFree;
}
Int_t AtTrack::GetTrackID()
{
   return fTrackID;
}
Double_t AtTrack::GetAngleZAxis()
{
   return fAngleZAxis;
}
Double_t AtTrack::GetAngleZDet()
{
   return fAngleZDet;
}
Double_t AtTrack::GetAngleYDet()
{
   return fAngleYDet;
}
XYZPoint AtTrack::GetTrackVertex()
{
   return fTrackVertex;
}
Double_t AtTrack::GetGeoTheta()
{
   return fGeoThetaAngle;
}
Double_t AtTrack::GetGeoPhi()
{
   return fGeoPhiAngle;
}
Bool_t AtTrack::GetIsNoise()
{
   return kIsNoise;
}
std::vector<Double_t> &AtTrack::GetRANSACCoeff()
{
   return fRANSACCoeff;
}
std::pair<Double_t, Double_t> AtTrack::GetGeoCenter()
{
   return fGeoCenter;
}
Double_t AtTrack::GetGeoRadius()
{
   return fGeoRadius;
}

void AtTrack::AddClusterHit(std::shared_ptr<AtHitCluster> hitCluster)
{
   fHitClusterArray.push_back(std::move(*hitCluster));
}

void AtTrack::ResetHitClusterArray()
{
   fHitClusterArray.clear();
}

XYZPoint AtTrack::GetLastPoint()
{
   Double_t maxR = 0.;
   XYZPoint maxPos, temp;
   for (Int_t nHit = 0; nHit < fHitArray.size(); nHit++) {
      temp = fHitArray.at(nHit).GetPosition();
      if (sqrt(pow(temp.X(), 2) + pow(temp.Y(), 2)) > maxR) {
         maxR = sqrt(pow(temp.X(), 2) + pow(temp.Y(), 2));
         maxPos = temp;
      }
   }
   return maxPos;
}
// alternative, but noticed that the last point in time is not alway the further away
/*
XYZPoint AtTrack::GetLastPoint()
{
   XYZPoint maxPos;
   if(fHitArray.size()>0){
      AtHit fhit = fHitArray.front(); // Last hit of the track (Low TB)
         AtHit lhit = fHitArray.back(); // First hit of the track (High TB)
         XYZPoint fhitPos = fhit.GetPosition();
         XYZPoint lhitPos = lhit.GetPosition();
         if( pow(fhitPos.X(),2) + pow(fhitPos.Y(),2) > pow(lhitPos.X(),2) + pow(lhitPos.Y(),2) ) maxPos = fhitPos;
      else maxPos = lhitPos;
   }
   return maxPos
}
*/

std::pair<Double_t, Double_t>
AtTrack::GetThetaPhi(const XYZPoint &vertex, const XYZPoint &maxPos, int zdir) // zdir -1 for simu // +1 for data
{
   std::pair<Double_t, Double_t> thetaPhi;
   if (fFitPar.size() > 0) {

      XYZPoint vp(TMath::Sign(1, maxPos.X()) * fabs(fFitPar[1]), TMath::Sign(1, maxPos.Y()) * fabs(fFitPar[3]),
                  zdir * TMath::Sign(1, (maxPos.Z() - vertex.Z())) * fabs(fFitPar[5])); // works with simu
      // XYZPoint
      // vp(TMath::Sign(1,maxPos.X())*fabs(fFitPar[1]),TMath::Sign(1,maxPos.Y())*fabs(fFitPar[3]),TMath::Sign(1,(maxPos.Z()-vertex.Z()))*fabs(fFitPar[5]));//works
      // with data
      //		std::cout<<" fFitPar "<<fFitPar[1]<<" "<<fFitPar[3]<<" "<<fFitPar[5]<<std::endl;
      //		std::cout<<" maxPos "<<maxPos.X()<<" "<<maxPos.Y()<<" "<<maxPos.Z()<<std::endl;

      thetaPhi.first = vp.Theta();
      thetaPhi.second = vp.Phi();
   }
   return thetaPhi;
}

Double_t AtTrack::GetMeanTime()
{
   Double_t meanTime = 0.0;

   if (fHitArray.size() > 0) {
      Int_t sum =
         std::accumulate(begin(fHitArray), end(fHitArray), 0, [](int i, AtHit &hit) { return hit.GetTimeStamp() + i; });
      return meanTime = sum / (Double_t)fHitArray.size();
   } else
      return meanTime;
}

Double_t AtTrack::GetLinearRange()
{
   if (fHitArray.size() > 0) {
      AtHit fhit = fHitArray.front(); // Last hit of the track (Low TB)
      AtHit lhit = fHitArray.back();  // First hit of the track (High TB)
      auto fhitPos = fhit.GetPosition();
      auto lhitPos = lhit.GetPosition();

      return TMath::Sqrt(TMath::Power((fhitPos.X() - lhitPos.X()), 2) + TMath::Power((fhitPos.Y() - lhitPos.Y()), 2) +
                         TMath::Power((fhitPos.Z() - lhitPos.Z()), 2));
   } else
      return 0;
}

Double_t AtTrack::GetLinearRange(XYZPoint vertex)
{

   if (fHitArray.size() > 0) {
      AtHit fhit = fHitArray.front();
      auto fhitPos = fhit.GetPosition();

      return TMath::Sqrt(TMath::Power((fhitPos.X() - vertex.X()), 2) + TMath::Power((fhitPos.Y() - vertex.Y()), 2) +
                         TMath::Power((fhitPos.Z() - vertex.Z()), 2));
   } else
      return 0;
}

Double_t AtTrack::GetLinearRange(const XYZPoint &vertex, const XYZPoint &maxPos)
{
   if (fHitArray.size() > 0) {
      return TMath::Sqrt(TMath::Power((maxPos.X() - vertex.X()), 2) + TMath::Power((maxPos.Y() - vertex.Y()), 2) +
                         TMath::Power((maxPos.Z() - vertex.Z()), 2));
   } else
      return 0;
}

Double_t AtTrack::GetGeoQEnergy()
{

   Double_t charge = 0;

   if (fHitArray.size() > 0) {
      for (Int_t i = 0; i < fHitArray.size(); i++) {
         charge += fHitArray.at(i).GetCharge();
      }

      return charge;

   } else
      return -10.0;
}


Bool_t AtTrack::SortHitArrayTime()
{
   std::sort(fHitArray.begin(), fHitArray.end(), AtHit::SortHitTime);
   return kTRUE;
}
