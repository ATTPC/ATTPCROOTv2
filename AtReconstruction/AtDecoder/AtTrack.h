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
   std::vector<AtHit> fHitArray;
   std::vector<AtHitCluster> fHitClusterArray; ///< Clusterized hits container
   Int_t fTrackID;
   std::vector<Double_t> fParFit;
   Double_t fMinimum;                  // Minimizer result
   Int_t fNFree;                       // Free paramets
   Double_t fAngleZAxis;               // Angle of the track with respecto to the X axis.
   Double_t fAngleZDet;                // Angle with respect to Z axis (beam axis) in the detector system.
   Double_t fAngleYDet;                //  "         "           Y   "             "
   Double_t fRange;                    // Range of the particle
   XYZPoint fTrackVertex;              // Mean Vertex of the track
   Double_t fGeoThetaAngle;            // Geometrical scattering angle with respect to the detector FitParameters
   Double_t fGeoPhiAngle;              //  " azimuthal "
   Double_t fGeoEnergy;                // Energy from the range
   Double_t fGeoQEnergy;               // Energy from the induced charge
   Int_t fQuadrant;                    // Pad plane quadrant
   std::vector<Double_t> fRANSACCoeff; // Coefficients for radius smoothing using RANSAC: x, y and radius of curvature
   Double_t fGeoRadius;                // Initial radius of curvature
   std::pair<Double_t, Double_t> fGeoCenter; // Center of the spiral track

   Bool_t kIsNoise;

public:
   AtTrack();
   AtTrack(const AtTrack &obj) = default;
   ~AtTrack() = default;

   void AddHit(const AtHit &hit);
   void AddClusterHit(std::shared_ptr<AtHitCluster> hitCluster);
   void SetTrackID(Int_t val);
   void SetFitPar(std::vector<Double_t> par);
   void SetMinimum(Double_t min); // Minimizer result
   void SetNFree(Int_t ndf);
   void SetAngleZAxis(Double_t angle);
   void SetAngleZDet(Double_t angle);
   void SetAngleYDet(Double_t angle);
   void SetTrackVertex(XYZPoint vertex);
   void SetRange(Double_t range);
   void SetGeoTheta(Double_t angle);
   void SetGeoPhi(Double_t angle);
   void SetGeoRange(Double_t range);
   void SetQuadrant(Int_t quad);
   void SetGeoEnergy(Double_t energy);
   void SetGeoQEnergy(Double_t qenergy);
   void SetIsNoise(Bool_t value);
   void SetRANSACCoeff(std::vector<Double_t> par);
   void SetGeoCenter(std::pair<Double_t, Double_t> center);
   void SetGeoRadius(Double_t radius);

   std::vector<AtHit> &GetHitArray() { return fHitArray; }
   std::vector<AtHitCluster> *GetHitClusterArray();
   void ResetHitClusterArray();
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
   Bool_t SortHitArrayTime();

   XYZPoint GetLastPoint();
   Double_t GetLinearRange(const XYZPoint &vertex, const XYZPoint &maxPos);
   std::pair<Double_t, Double_t> GetThetaPhi(const XYZPoint &vertex, const XYZPoint &maxPos, int zdir);

   friend inline std::ostream &operator<<(std::ostream &o, const AtTrack &track)
   {
      std::cout << cYELLOW << " ====================================================== " << std::endl;
      std::cout << "  Track " << track.fTrackID << " Info : " << std::endl;
      std::cout << " Quadrant : " << track.fQuadrant << std::endl;
      std::cout << " Geomterical Scattering Angle : " << track.fGeoThetaAngle * (180.0 / TMath::Pi()) << " deg "
                << " - Geomterical Azimuthal Angle : " << track.fGeoPhiAngle * (180.0 / TMath::Pi()) << " deg "
                << std::endl;
      std::cout << " Geometrical Range : " << track.fRange << " mm " << std::endl;
      std::cout << " Angle with respect to Z axis : " << track.fAngleZAxis << cNORMAL << std::endl;

      return o;
   }

   ClassDef(AtTrack, 2);
};

#endif
