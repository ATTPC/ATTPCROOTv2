#ifndef ATTRACK_H
#define ATTRACK_H

#include "AtContainerManip.h"
#include "AtHit.h"
#include "AtHitCluster.h"
#include "AtPattern.h" // IWYU pragma: keep

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <Rtypes.h>
#include <TMath.h>
#include <TObject.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

class AtTrack : public TObject {

protected:
   using XYZPoint = ROOT::Math::XYZPoint;
   using HitPtr = std::unique_ptr<AtHit>;
   using HitVector = std::vector<HitPtr>;
   using PatternPtr = std::unique_ptr<AtPatterns::AtPattern>;

   // Attributes shared by all track finding algorithms
   Int_t fTrackID{-1};
   HitVector fHitArray;
   PatternPtr fPattern{nullptr};
   Bool_t fIsMerged{false};
   Double_t fVertexToZDist{0};

   // Used by AtPRA
   Double_t fGeoThetaAngle{};                //< Geometrical scattering angle with respect to the detector FitParameters
   Double_t fGeoPhiAngle{};                  //<  " azimuthal "
   Double_t fGeoRadius{};                    //< Initial radius of curvature
   std::pair<Double_t, Double_t> fGeoCenter; //< Center of the spiral track
   std::vector<AtHitCluster> fHitClusterArray; //< Clusterized hits container. Can also be stored in fHitArray

public:
   AtTrack() = default;
   AtTrack(const AtTrack &obj);
   AtTrack &operator=(AtTrack obj);
   AtTrack(AtTrack &&) = default;
   ~AtTrack() = default;

   /**
    * @brief ADL-findable swap for AtTrack.
    *
    * Implemented as a friend free function to enable ADL of swap.
    * (https://en.cppreference.com/w/cpp/language/adl)
    */
   friend void swap(AtTrack &a, AtTrack &b) noexcept
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
   };

   // Getters
   Int_t GetTrackID() const { return fTrackID; }
   HitVector &GetHitArray() { return fHitArray; }
   const HitVector &GetHitArray() const { return fHitArray; }

   std::vector<AtHit> GetHitArrayObject() { return ContainerManip::GetObjectVector(fHitArray); }
   // const std::vector<AtHit> &GetHitArrayConst() const { return fHitArray; }
   const AtPatterns::AtPattern *GetPattern() const { return fPattern.get(); }

   Double_t GetGeoTheta() const { return fGeoThetaAngle; }
   Double_t GetGeoPhi() const { return fGeoPhiAngle; }
   Double_t GetGeoRadius() const { return fGeoRadius; }
   std::pair<Double_t, Double_t> GetGeoCenter() const { return fGeoCenter; }
   std::vector<AtHitCluster> *GetHitClusterArray() { return &fHitClusterArray; }

   Bool_t GetIsMerged() const { return fIsMerged; }
   Double_t GetVertexToZDist() const { return fVertexToZDist; }

   // Setters
   void SetTrackID(Int_t val) { fTrackID = val; }
   void AddHit(const AtHit &hit) { fHitArray.emplace_back(std::make_unique<AtHit>(hit)); }
   void AddHit(HitPtr hit) { fHitArray.push_back(std::move(hit)); }
   void SetPattern(std::unique_ptr<AtPatterns::AtPattern> pat) { fPattern = std::move(pat); }

   void SetGeoTheta(Double_t angle) { fGeoThetaAngle = angle; }
   void SetGeoPhi(Double_t angle) { fGeoPhiAngle = angle; }
   void SetGeoRadius(Double_t radius) { fGeoRadius = radius; }
   void SetGeoCenter(std::pair<Double_t, Double_t> center) { fGeoCenter = center; }
   void AddClusterHit(std::shared_ptr<AtHitCluster> hitCluster);

   void SetIsMerged(bool val) { fIsMerged = val; }
   void SetVertexToZDist(Double_t val) { fVertexToZDist = val; }

   // Operations
   void ResetHitClusterArray() { fHitClusterArray.clear(); }

   Double_t GetMeanTime();
   Double_t GetLinearRange();
   Double_t GetLinearRange(XYZPoint vertex);

   Double_t GetGeoQEnergy();
   XYZPoint GetLastPoint();
   XYZPoint GetFirstPoint();
   static Double_t GetLinearRange(const XYZPoint &vertex, const XYZPoint &maxPos);

   void SortHitArrayTime();
   void SortClusterHitArrayZ();

protected:
   static Bool_t SortClusterHitZ(const AtHitCluster &lhs, const AtHitCluster &rhs)
   {
      return lhs.GetPosition().Z() < rhs.GetPosition().Z();
   }

   friend inline std::ostream &operator<<(std::ostream &o, const AtTrack &track)
   {
      std::cout << " ====================================================== " << std::endl;
      std::cout << "  Track " << track.fTrackID << " Info : " << std::endl;
      std::cout << " Geomterical Scattering Angle : " << track.fGeoThetaAngle * (180.0 / TMath::Pi()) << " deg "
                << " - Geomterical Azimuthal Angle : " << track.fGeoPhiAngle * (180.0 / TMath::Pi()) << " deg "
                << std::endl;

      return o;
   }

   ClassDef(AtTrack, 3);
};

#endif
