#include "AtTrack.h"

#include "AtPattern.h"

#include <Rtypes.h>
#include <TMath.h>
#include <TMathBase.h>

#include <cmath>
#include <iterator>
#include <numeric>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

ClassImp(AtTrack);

AtTrack::AtTrack(const AtTrack &obj)
{
   fTrackID = obj.fTrackID;
   fHitArray = obj.fHitArray;

   fPattern = (obj.fPattern != nullptr) ? obj.fPattern->Clone() : nullptr;

   fGeoThetaAngle = obj.fGeoThetaAngle;
   fGeoPhiAngle = obj.fGeoPhiAngle;
   fGeoRadius = obj.fGeoRadius;
   fGeoCenter = obj.fGeoCenter;
   fHitClusterArray = obj.fHitClusterArray;
}

void AtTrack::AddClusterHit(std::shared_ptr<AtHitCluster> hitCluster)
{
   fHitClusterArray.push_back(std::move(*hitCluster));
}

XYZPoint AtTrack::GetLastPoint()
{
   Double_t maxR = 0.;
   XYZPoint maxPos, temp;
   for (auto &nHit : fHitArray) {
      temp = nHit.GetPosition();
      if (sqrt(pow(temp.X(), 2) + pow(temp.Y(), 2)) > maxR) {
         maxR = sqrt(pow(temp.X(), 2) + pow(temp.Y(), 2));
         maxPos = temp;
      }
   }
   return maxPos;
}

Double_t AtTrack::GetMeanTime()
{
   Double_t meanTime = 0.0;

   if (fHitArray.size() > 0) {
      Int_t sum =
         std::accumulate(begin(fHitArray), end(fHitArray), 0, [](int i, AtHit &hit) { return hit.GetTimeStamp() + i; });
      return sum / (Double_t)fHitArray.size();
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
   return TMath::Sqrt(TMath::Power((maxPos.X() - vertex.X()), 2) + TMath::Power((maxPos.Y() - vertex.Y()), 2) +
                      TMath::Power((maxPos.Z() - vertex.Z()), 2));
}

Double_t AtTrack::GetGeoQEnergy()
{

   Double_t charge = 0;

   if (fHitArray.size() > 0) {
      return std::accumulate(begin(fHitArray), end(fHitArray), 0.0,
                             [](double i, const AtHit &hit) { return hit.GetCharge() + i; });
   } else
      return -10.0;
}

void AtTrack::SortHitArrayTime()
{
   std::sort(fHitArray.begin(), fHitArray.end(), AtHit::SortHitTime);
}

void AtTrack::SortClusterHitArrayZ()
{

   std::sort(fHitClusterArray.begin(), fHitClusterArray.end(), SortClusterHitZ);
}
