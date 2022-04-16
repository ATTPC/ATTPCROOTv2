/*******************************************************************
// Basic Lmeds Class                                              *
// Author: J.C. Zamora, jczamorac@gmail.com                        *
// University of Sao Paulo, 26-08-2020                             *
********************************************************************/

#ifndef AtLmedsMOD_H
#define AtLmedsMOD_H

#ifdef _OPENMP
#include <omp.h>
#endif

#include "AtTrack.h" // for AtTrack

#include <Rtypes.h>   // for Int_t, Double_t, THashConsistencyHolder, ClassDef
#include <TObject.h>  // for TObject
#include <TVector3.h> // for TVector3

#include <stdio.h> // for size_t

#include <algorithm> // for max
#include <utility>   // for pair
#include <vector>    // for vector
class AtEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class AtLmedsMod : public TObject {

public:
   AtLmedsMod();
   ~AtLmedsMod();

   void Reset();
   void Init(AtEvent *event);
   void Solve();
   std::vector<int> RandSam(std::vector<int> indX, Int_t mode);
   void EstimModel(const std::vector<int> samplesIdx);
   double EstimError(int i);
   void CalcLmedsMod(AtEvent *event);
   std::vector<double> GetChargeOfTracks();
   std::vector<double> GetTrackLength();
   std::vector<double> GetPDF(const std::vector<int> samplesIdx);
   void SetAvCharge(double charge) { Avcharge = charge; };
   double GetAvCharge() { return Avcharge; };
   double Fit3D(std::vector<int> inliners, TVector3 &V1, TVector3 &V2);
   void SetDistanceThreshold(Float_t threshold) { fLmedsThreshold = threshold; };
   void SetMinHitsLine(Int_t nhits) { fLmedsMinPoints = nhits; };
   void SetNumItera(Int_t niterations) { fLmedsMaxIteration = niterations; };
   TVector3 GetVertex1() { return fVertex_1; };
   TVector3 GetVertex2() { return fVertex_2; };
   Double_t GetVertexTime() { return fVertexTime; };
   TVector3 GetVertexMean() { return fVertex_mean; };
   std::vector<AtTrack> GetTrackCand() { return fTrackCand; };
   double GetMedian(std::vector<double> errvec);
   TVector3 ClosestPoint2Lines(TVector3 d1, TVector3 pt1, TVector3 d2, TVector3 pt2);
   void SetRanSamMode(Int_t mode) { fRandSamplMode = mode; };
   void SetChargeThres(double value) { fChargeThres = value; };
   void SetVertexMod(Int_t mode) { fVertexMod = mode; };

   struct Cluster // return type of structure
   {
      double ClusterStrength;        // strength
      size_t ClusterSize;            // size
      double ClusterChi2;            // Chi2
      std::vector<int> ClusterIndex; // Indices
      TVector3 ClusterFitP1;         // point 1 from the fitted line
      TVector3 ClusterFitP2;         // point 2 from the fitted line
   };

   typedef std::vector<Cluster> AllClusters;
   std::vector<AtTrack *> Clusters2Tracks(AllClusters NClusters, AtEvent *event);

   void SetCluster(const std::vector<int> samplesIdx, const double cost, const double Chi2, TVector3 CP1, TVector3 CP2);
   inline AllClusters GetClusters() { return cluster_vector; }
   AllClusters cluster_vector;

protected:
   void FindVertex(std::vector<AtTrack *> tracks);
   void FindVertexOneTrack(std::vector<AtTrack *> tracks);

   std::vector<double> errorsVec;
   TVector3 fVertex_1;
   TVector3 fVertex_2;
   TVector3 fVertex_mean;
   Double_t fVertexTime;
   Double_t fMinimum;
   std::vector<AtTrack> fTrackCand;        // Candidate tracks
   std::pair<Int_t, Int_t> fVertex_tracks; // ID of the tracks that form the best vertex
   Int_t fLineDistThreshold;
   Int_t fRandSamplMode;
   Int_t fVertexMod{};

   std::vector<double> vX, vY, vZ, vQ;
   std::vector<double> vTrackCharge;
   float fLmedsMinPoints;
   float fLmedsPointThreshold{};
   float fLmedsChargeThreshold{};
   float fLmedsThreshold;
   float fLmedsMaxIteration;
   int fNumberOfTracksMax{};
   int fOriginalCloudSize{};
   double fTotalCharge{};
   int fVerbose{};
   double Avcharge{};
   double fChargeThres;

public:
   TVector3 Vs;
   TVector3 Ps;

   ClassDef(AtLmedsMod, 1);
};

#endif
