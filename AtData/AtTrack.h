#ifndef ATTRACK_H
#define ATTRACK_H

#include "TROOT.h"
#include "TObject.h"
#include "TMath.h"
#include "Math/Point3D.h"

#include <numeric>
#include <algorithm>
#include <iostream>

// AtTPCROOT
#include "AtHit.h"
#include "AtHitCluster.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"
using XYZPoint = ROOT::Math::XYZPoint;

class AtTrack : public TObject {

protected:
   // Attributes shared by all track finding algorithms
   Int_t fTrackID;
   std::vector<AtHit> fHitArray; // TrackHC, AtGenfit, all ransacs

   // Used by all ransac classes
   XYZPoint fTrackVertex;         // Mean Vertex of the track
   Double_t fMinimum;             // Minimizer result (chi2)
   Int_t fNFree;                  // Free paramets (NDF)
   std::vector<Double_t> fFitPar; // fit parameters

   // Used by AtRansac only
   Double_t fAngleZAxis;               // Angle of the track w.r.t. the X axis.
   Double_t fAngleZDet;                // Angle w.r.t. Z axis (beam axis) in the detector system.
   Double_t fAngleYDet;                //  "         "           Y   "             "
   std::vector<Double_t> fRANSACCoeff; // Coefficients for radius smoothing using RANSAC: x, y and radius of curvature

   // Used by AtPRA
   Double_t fGeoThetaAngle;                  // Geometrical scattering angle with respect to the detector FitParameters
   Double_t fGeoPhiAngle;                    //  " azimuthal "
   Double_t fGeoRadius;                      // Initial radius of curvature
   std::pair<Double_t, Double_t> fGeoCenter; // Center of the spiral track
   std::vector<AtHitCluster> fHitClusterArray; ///< Clusterized hits container

   // Used by AtTrackFinderHC
   Bool_t kIsNoise;

public:
   AtTrack();
   AtTrack(const AtTrack &obj) = default;
   ~AtTrack() = default;

   // Attributes shared by all track finding algorithms
   void SetTrackID(Int_t val) { fTrackID = val; }
   void AddHit(const AtHit &hit) { fHitArray.push_back(hit); }

   // Atributes used by all ransac classes

   void SetFitPar(const std::vector<Double_t> &par) { fFitPar = par; }
   void SetMinimum(Double_t min) { fMinimum = min; }
   void SetNFree(Int_t ndf);
   void SetTrackVertex(XYZPoint vertex);

   // Attributes used by AtRansac
   void SetAngleZAxis(Double_t angle);
   void SetAngleZDet(Double_t angle);
   void SetAngleYDet(Double_t angle);
   void SetRANSACCoeff(std::vector<Double_t> par);

   // Attributes used by AtPRAtask
   void SetGeoTheta(Double_t angle);
   void SetGeoPhi(Double_t angle);
   void AddClusterHit(std::shared_ptr<AtHitCluster> hitCluster);
   void SetGeoCenter(std::pair<Double_t, Double_t> center);
   void SetGeoRadius(Double_t radius);

   // Attributes used by AtTrackFinderHC
   void SetIsNoise(Bool_t value);

   std::vector<AtHit> &GetHitArray() { return fHitArray; }
   std::vector<AtHitCluster> *GetHitClusterArray();
   void ResetHitClusterArray() { fHitClusterArray.clear(); }
   // void ResetHitArray();
   std::vector<Double_t> GetFitPar();
   const std::vector<AtHit> &GetHitArrayConst() const { return fHitArray; }
   Double_t GetMinimum();
   Int_t GetNFree();
   Int_t GetTrackID();
   Double_t GetAngleZAxis();
   Double_t GetAngleZDet();
   Double_t GetAngleYDet();
   Double_t GetMeanTime();
   Double_t GetLinearRange();
   Double_t GetLinearRange(XYZPoint vertex);
   Double_t GetLinearRange(const XYZPoint &vertex, const XYZPoint &maxPos);
   XYZPoint GetTrackVertex();
   Int_t GetQuadrant();
   Double_t GetGeoTheta();
   Double_t GetGeoPhi();
   Double_t GetGeoEnergy();
   Double_t GetGeoQEnergy();
   Bool_t GetIsNoise();
   std::vector<Double_t> &GetRANSACCoeff();
   std::pair<Double_t, Double_t> GetGeoCenter();
   Double_t GetGeoRadius();
   XYZPoint GetLastPoint();
   std::pair<Double_t, Double_t> GetThetaPhi(const XYZPoint &vertex, const XYZPoint &maxPos, int zdir);

   void SortHitArrayTime();
   void SortClusterHitArrayZ();

protected:
   static Bool_t SortClusterHitZ(const AtHitCluster &lhs, const AtHitCluster &rhs)
   {
      return lhs.GetPosition().Z() < rhs.GetPosition().Z();
   }

   friend inline std::ostream &operator<<(std::ostream &o, const AtTrack &track)
   {
      std::cout << cYELLOW << " ====================================================== " << std::endl;
      std::cout << "  Track " << track.fTrackID << " Info : " << std::endl;
      std::cout << " Geomterical Scattering Angle : " << track.fGeoThetaAngle * (180.0 / TMath::Pi()) << " deg "
                << " - Geomterical Azimuthal Angle : " << track.fGeoPhiAngle * (180.0 / TMath::Pi()) << " deg "
                << std::endl;
      std::cout << " Angle with respect to Z axis : " << track.fAngleZAxis << cNORMAL << std::endl;

      return o;
   }

   ClassDef(AtTrack, 2);
};

#endif
