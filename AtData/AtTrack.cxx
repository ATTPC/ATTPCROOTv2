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

/**
 * @brief ADL-findable swap for AtTrack.
 *
 * Implemented as a friend free function to enable ADL of swap.
 * (https://en.cppreference.com/w/cpp/language/adl)
 */
void swap(AtTrack &a, AtTrack &b) noexcept
{
   using std::swap; // Enable ADL

   swap(a.fTrackID, b.fTrackID);
   swap(a.fHitArray, b.fHitArray);
   swap(a.fHitClusterArray, b.fHitClusterArray);
   swap(a.fPattern, b.fPattern);
   swap(a.fIsMerged, b.fIsMerged);
   swap(a.fVertexToZDist, b.fVertexToZDist);

   swap(a.fGeoThetaAngle, b.fGeoThetaAngle);
   swap(a.fGeoPhiAngle, b.fGeoPhiAngle);
   swap(a.fGeoRadius, b.fGeoRadius);
   swap(a.fGeoCenter, b.fGeoCenter);
}

/**
 * @brief Copy assignment using copy-swap idiom.
 *
 */
AtTrack &AtTrack::operator=(AtTrack obj)
{
   swap(*this, obj);
   return *this;
}
AtTrack::AtTrack(const AtTrack &obj)
{
   fTrackID = obj.fTrackID;
   fHitArray = obj.fHitArray;
   fHitClusterArray = obj.fHitClusterArray;
   fPattern = (obj.fPattern != nullptr) ? obj.fPattern->Clone() : nullptr;
   fIsMerged = obj.fIsMerged;
   fVertexToZDist = obj.fVertexToZDist;

   fGeoThetaAngle = obj.fGeoThetaAngle;
   fGeoPhiAngle = obj.fGeoPhiAngle;
   fGeoRadius = obj.fGeoRadius;
   fGeoCenter = obj.fGeoCenter;
}

void AtTrack::AddClusterHit(std::shared_ptr<AtHitCluster> hitCluster)
{
   fHitClusterArray.push_back(std::move(*hitCluster));
}

XYZPoint AtTrack::GetLastPoint()
{
   Double_t maxR = 0.;
   XYZPoint maxPos;
   for (auto &nHit : fHitArray) {
      auto temp = nHit.GetPosition();
      if (temp.Rho() > maxR) {
         maxR = temp.Rho();
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
      return GetLinearRange(fhit.GetPosition(), lhit.GetPosition());
   } else
      return 0;
}

Double_t AtTrack::GetLinearRange(XYZPoint vertex)
{
   if (fHitArray.size() > 0) {
      AtHit fhit = fHitArray.front();
      return GetLinearRange(fhit.GetPosition(), vertex);
   } else
      return 0;
}

Double_t AtTrack::GetLinearRange(const XYZPoint &vertex, const XYZPoint &maxPos)
{
   return (vertex - maxPos).R();
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
