#include "AtRansacMod.h"

// FairRoot classes
#include <FairRuntimeDb.h>
#include <FairRun.h>

#include <iostream>

using namespace std;

ClassImp(AtRansacMod);

AtRansacMod::AtRansacMod()
{

   fRANSACMaxIteration = 500;
   fRANSACMinPoints = 30;
   fRANSACThreshold = 15;
   fLineDistThreshold = 40.0; // 6.
   fRandSamplMode = 0;

   fVertex_1.SetXYZ(-10000, -10000, -10000);
   fVertex_2.SetXYZ(-10000, -10000, -10000);
   fVertexTime = -1000.0;
   fMinimum = -1.0;
   fChargeThres = 0;
   // fVertexMod = 0;
}

AtRansacMod::~AtRansacMod() {}

void AtRansacMod::Init(AtEvent *event)
{

   Reset();

   auto hitArray = event->GetHitArray();
   Int_t nHits = hitArray.size();
   // std::cout << "Init   Hitarray size    "<<nHits << '\n';

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      AtHit hit = hitArray.at(iHit);
      auto position = hit.GetPosition();
      Double_t tq = hit.GetCharge();
      if (tq < fChargeThres)
         continue;
      vX.push_back(position.X()); // in cm
      vY.push_back(position.Y()); // in cm
      vZ.push_back(position.Z()); // in cm
      vQ.push_back(tq);
   }

   fOriginalCloudSize = vX.size();
   double TotalCharge = 0;
   for (unsigned int i = 0; i < vQ.size(); i++) {
      TotalCharge += vQ[i];
   }
   fTotalCharge = TotalCharge;
}

void AtRansacMod::Reset()
{

   vX.clear();
   vY.clear();
   vZ.clear();
   vQ.clear();
   Vs.Clear();
   Ps.Clear();
   cluster_vector.clear();
}

vector<double> AtRansacMod::GetPDF(const std::vector<int> samplesIdx)
{

   size_t pclouds = samplesIdx.size();
   double Tcharge = 0;
   for (int i = 0; i < pclouds; i++)
      Tcharge += vQ[samplesIdx[i]];

   SetAvCharge(Tcharge / pclouds);
   std::vector<double> w;
   if (Tcharge > 0)
      for (int i = 0; i < pclouds; i++)
         w.push_back(vQ[samplesIdx[i]] / Tcharge);

   return w;
}

vector<int> AtRansacMod::RandSam(vector<int> indX, Int_t mode)
{
   size_t pclouds = indX.size();
   std::vector<double> Proba = GetPDF(indX);
   int p1, p2;
   double w1, w2;
   vector<int> ranpair;
   ranpair.resize(2);

   if (mode == 0) {
      //-------Uniform sampling
      p1 = (int)(gRandom->Uniform(0, pclouds));

      do {
         p2 = (int)(gRandom->Uniform(0, pclouds));
      } while (p2 == p1);

      ranpair[0] = indX[p1];
      ranpair[1] = indX[p2];
   }

   if (mode == 1) {
      //--------Gaussian sampling
      double dist = 0;
      double sigma = 30.0;
      double y = 0;
      double gauss = 0;
      int counter = 0;
      p1 = (int)(gRandom->Uniform(0, pclouds));
      TVector3 P1 = {vX[indX[p1]], vY[indX[p1]], vZ[indX[p1]]};
      do {
         p2 = (int)(gRandom->Uniform(0, pclouds));
         TVector3 P2 = {vX[indX[p2]], vY[indX[p2]], vZ[indX[p2]]};
         TVector3 dif = P2 - P1;
         dist = dif.Mag();
         gauss = 1.0 * exp(-1.0 * pow(dist / sigma, 2.0));
         y = (gRandom->Uniform(0, 1));
         counter++;
         if (counter > 20 && p2 != p1)
            break;
      } while (p2 == p1 || y > gauss);

      ranpair[0] = indX[p1];
      ranpair[1] = indX[p2];
   }

   if (mode == 2) {
      //-------Weighted sampling
      bool cond = false;
      int counter = 0;
      p1 = (int)(gRandom->Uniform(0, pclouds));
      do {
         counter++;
         if (counter > 30 && p2 != p1)
            break;
         p2 = (int)(gRandom->Uniform(0, pclouds));
         cond = false;
         double TwiceAvCharge = 2 * GetAvCharge();
         if (Proba.size() == pclouds) {
            w2 = gRandom->Uniform(0, TwiceAvCharge);
            if (Proba[p2] >= w2)
               cond = true;
         } else {
            w2 = 1;
            cond = true;
         }
      } while (p2 == p1 || cond == false);

      ranpair[0] = indX[p1];
      ranpair[1] = indX[p2];
   }

   if (mode == 3) {
      //-------Weighted sampling + Gauss dist.
      bool cond = false;
      double dist = 0;
      double sigma = 30.0;
      double y = 0;
      double gauss = 0;
      int counter = 0;
      p1 = (int)(gRandom->Uniform(0, pclouds));
      TVector3 P1 = {vX[indX[p1]], vY[indX[p1]], vZ[indX[p1]]};
      do {
         p2 = (int)(gRandom->Uniform(0, pclouds));
         TVector3 P2 = {vX[indX[p2]], vY[indX[p2]], vZ[indX[p2]]};
         TVector3 dif = P2 - P1;
         dist = dif.Mag();
         gauss = 1.0 * exp(-1.0 * pow(dist / sigma, 2));
         y = (gRandom->Uniform(0, 1));
         counter++;
         if (counter > 30 && p2 != p1)
            break;

         cond = false;
         double TwiceAvCharge = 2 * GetAvCharge();
         if (Proba.size() == pclouds) {
            w2 = gRandom->Uniform(0, TwiceAvCharge);
            if (Proba[p2] >= w2)
               cond = true;
         } else {
            w2 = 1;
            cond = true;
         }

      } while (p2 == p1 || cond == false || y > gauss);

      ranpair[0] = indX[p1];
      ranpair[1] = indX[p2];
   }

   return ranpair;
}

void AtRansacMod::EstimModel(const std::vector<int> samplesIdx)
{

   // line from two points
   TVector3 Po1 = {vX[samplesIdx[0]], vY[samplesIdx[0]], vZ[samplesIdx[0]]};
   TVector3 Po2 = {vX[samplesIdx[1]], vY[samplesIdx[1]], vZ[samplesIdx[1]]};

   Vs = Po2 - Po1;
   Ps = Po1;
}

double AtRansacMod::EstimError(int i)
{
   // distance point to line
   TVector3 newPoint = {vX[i], vY[i], vZ[i]};
   TVector3 vec = Ps - newPoint;
   TVector3 nD = Vs.Cross(vec);
   double dist = nD.Mag() / Vs.Mag();

   return dist;
}

void AtRansacMod::Solve()
{

   // std::cout << "numero de puntos  "<<vX.size()<< '\n';
   std::vector<int> remainIndex;
   for (size_t i = 0; i < vX.size(); i++)
      remainIndex.push_back(i);

   TVector3 V1, V2;
   std::vector<int> inliners;
   inliners.clear();
   std::vector<std::pair<double, int>> IdxMod1;
   std::vector<std::pair<double, int>> IdxMod2;

   for (int i = 0; i < fRANSACMaxIteration; i++) {

      if (remainIndex.size() < fRANSACMinPoints)
         break;

      std::vector<int> Rsamples = RandSam(remainIndex, fRandSamplMode); // random sampling
      EstimModel(Rsamples);                                             // estimate the linear model

      // std::vector<int> inlIdxR;
      int nbInliers = 0;
      double weight = 0;

      for (auto j = remainIndex.begin(); j != remainIndex.end(); ++j) {

         double error = EstimError(*j); // error of each point relative to the model
         error = error * error;
         if (error < (fRANSACThreshold * fRANSACThreshold)) {
            // inlIdxR.push_back(*j);
            nbInliers++;
            weight += error;
         }
      }

      if (nbInliers > fRANSACMinPoints) {
         // getting the best models
         double scale = weight / nbInliers;
         IdxMod1.push_back(std::make_pair(scale, Rsamples[0]));
         IdxMod2.push_back(std::make_pair(scale, Rsamples[1]));
      } // if a cluster was found

   } // for RANSAC interactions

   // sort clusters
   sort(IdxMod1.begin(), IdxMod1.end());
   sort(IdxMod2.begin(), IdxMod2.end());

   remainIndex.clear(); // track remaining points
   for (size_t i = 0; i < vX.size(); i++)
      remainIndex.push_back(i);

   // extract inliers using the models
   for (int i = 0; i < IdxMod1.size(); ++i) {
      std::vector<int> ModInx = {IdxMod1[i].second, IdxMod2[i].second};
      EstimModel(ModInx);
      std::vector<int> inlIdxR;
      // inlIdxR.clear();
      ModInx.clear();

      if (remainIndex.size() < fRANSACMinPoints)
         break;

      int counter = 0;

      for (auto j = remainIndex.begin(); j != remainIndex.end(); ++j) {
         double error = EstimError(*j);

         if ((error * error) < (fRANSACThreshold * fRANSACThreshold)) {
            inlIdxR.push_back(*j);
            counter++;
         }
      }

      if (counter > fRANSACMinPoints) {
         TVector3 v1, v2;
         double chi2 = Fit3D(inlIdxR, v1, v2);
         SetCluster(inlIdxR, IdxMod1[i].first, chi2, v1, v2);
         v1.Clear();
         v2.Clear();
      }
      std::vector<int> tempRemain;
      std::set_difference(remainIndex.begin(), remainIndex.end(), inlIdxR.begin(), inlIdxR.end(),
                          std::inserter(tempRemain, tempRemain.begin()));
      remainIndex = tempRemain;
      inlIdxR.clear();
      tempRemain.clear();
   }

   IdxMod1.clear();
   IdxMod2.clear();
   remainIndex.clear();
}

void AtRansacMod::CalcRANSACMod(AtEvent *event)
{

   // std::cout << "Inicializa" << '\n';
   Init(event);
   // std::cout << "Resuelve" << '\n';
   if (vX.size() > fRANSACMinPoints) {
      Solve();
      AllClusters myClusters = GetClusters();
      // I know this step is stupid, but this part is meant to be in the future a clustering process
      // std::cout << "Escribe tracks" << '\n';
      std::vector<AtTrack *> tracks = Clusters2Tracks(myClusters, event);

      Int_t tracksSize = tracks.size();
      // std::cout<<"RansacMod tracks size : "<<tracksSize<<std::endl;
      for (Int_t ntrack = 0; ntrack < tracks.size(); ntrack++) {
         tracks.at(ntrack)->SetTrackID(ntrack);
         // std::vector<Double_t> p = tracks.at(ntrack)->GetFitPar();
         // std::cout << "TrackID   "<<tracks.at(ntrack)->GetTrackID() << '\n';
         // std::cout << "Hits in track   "<<tracks.at(ntrack)->GetHitArray()->at(0).GetHitID() << '\n';
         // if(p.size()!=0){
         // tracks.at(ntrack)->SetAngleZAxis(angZDeti);
         // std::cout << "/* Un nuevo track candidato! */" << '\n';
         // fTrackCand.push_back(*tracks.at(ntrack));
         //}
      }
      switch (fVertexMod) {
      case 1:
         if (tracksSize > 1)
            FindVertex(tracks);
         break;
      default:
         if (tracksSize > 0)
            FindVertexOneTrack(tracks); // find a vertex for each track
      }
   }
}

void AtRansacMod::SetCluster(const std::vector<int> samplesIdx, const double cost, const double Chi2, TVector3 CP1,
                             TVector3 CP2)
{
   Cluster cstr;
   cstr.ClusterIndex = samplesIdx;
   cstr.ClusterSize = samplesIdx.size();
   cstr.ClusterStrength = cost;
   cstr.ClusterChi2 = Chi2;
   cstr.ClusterFitP1 = CP1;
   cstr.ClusterFitP2 = CP2;
   cluster_vector.push_back(cstr);
}

std::vector<AtTrack *> AtRansacMod::Clusters2Tracks(AllClusters NClusters, AtEvent *event)
{

   std::vector<AtTrack *> tracks;

   auto hits = event->GetHitArray();

   int numclus = NClusters.size();
   // std::cout << "numero de clusters "<<numclus << '\n';

   for (int i = 0; i < numclus; i++) {
      size_t clustersize = NClusters[i].ClusterSize;
      std::vector<int> indicesCluster = NClusters[i].ClusterIndex;
      // double costo =  NClusters[i].ClusterStrength;
      double Chi2 = NClusters[i].ClusterChi2;
      TVector3 punto1 = NClusters[i].ClusterFitP1;
      TVector3 punto2 = NClusters[i].ClusterFitP2;
      TVector3 pdiff = punto2 - punto1;
      AtTrack *track = new AtTrack();
      // std::cout << "hits en el cluster   "<<clustersize<<"   "<<indicesCluster.size() << '\n';
      for (int j = 0; j < clustersize; j++) {
         track->AddHit(hits.at(indicesCluster[j]));
         // std::cout << j <<"  "<<indicesCluster[j]<< '\n';
      }
      // std::cout << "hits en el track   "<<track->GetHitArray()->size()<<"   "<<
      // track->GetHitArray()->at(0).GetTrackID()<< '\n';
      std::vector<Double_t> par;
      par.push_back(punto1.X()); // 0
      par.push_back(pdiff.X());  // 1
      par.push_back(punto1.Y()); // 2
      par.push_back(pdiff.Y());  // 3
      par.push_back(punto1.Z()); // 4
      par.push_back(pdiff.Z());  // 5

      track->SetFitPar(par);
      track->SetMinimum(Chi2);
      track->SetNFree(clustersize - 6);

      tracks.push_back(track);

      // delete track;
   }

   return tracks;
}

void AtRansacMod::FindVertex(std::vector<AtTrack *> tracks)
{

   // Assumes the minimum distance between two lines, with respect a given threshold, the first vertex candidate. Then
   // evaluates the distance of each remaining line with respect to the others (vertex) to decide the particles of the
   // reaction.
   // std::cout<<" New find vertex call "<<std::endl;

   Double_t mad = 10; // Minimum approach distance. This is the minimum distance between the lines in 3D. Must be bigger
                      // than fLineDistThreshold
   // ROOT::Math::XYZVector c_1(-1000,-1000,-1000);
   // ROOT::Math::XYZVector c_2(-1000,-1000,-1000);
   TVector3 c_1(-1000, -1000, -1000);
   TVector3 c_2(-1000, -1000, -1000);
   // std::vector<AtTrack*> *TrackCand;

   // Current  parametrization
   // x = p[0] + p[1]*t;
   // y = p[2] + p[3]*t;
   // z = p[4] + p[5]*t;
   //  (x,y,z) = (p[0],p[2],p[4]) + (p[1],p[3],p[5])*t

   // Vector of the beam determined from the experimental data
   TVector3 BeamDir(0., 0., 1.0);
   TVector3 BeamPoint(0., 0., 500.0);

   std::vector<Bool_t> IsFilled;
   for (Int_t i = 0; i < int(tracks.size()); i++)
      IsFilled.push_back(kFALSE);

   // Test each line against the others to find a vertex candidate
   for (Int_t i = 0; i < int(tracks.size()) - 1; i++) {

      AtTrack *track = tracks.at(i);
      std::vector<Double_t> p = track->GetFitPar();

      if (p.size() == 0)
         continue;

      TVector3 p1(p[0], p[2], p[4]); // p1
      TVector3 e1(p[1], p[3], p[5]); // d1

      for (Int_t j = i + 1; j < tracks.size(); j++) {
         AtTrack *track_f = tracks.at(j);
         std::vector<Double_t> p_f = track_f->GetFitPar();
         if (track->GetTrackID() == track_f->GetTrackID())
            continue;

         if (p_f.size() == 0)
            continue;

         TVector3 p2(p_f[0], p_f[2], p_f[4]); // p2
         TVector3 e2(p_f[1], p_f[3], p_f[5]); // d2
         double angle = e1.Angle(e2) * 180. / 3.1415;

         TVector3 n = e1.Cross(e2);
         double sdist = fabs(n.Dot(p1 - p2) / n.Mag());
         TVector3 vertexbuff = ClosestPoint2Lines(e1, p1, e2, p2);

         if (sdist < fLineDistThreshold) {
            TVector3 meanVer = vertexbuff;
            double radius = sqrt(vertexbuff.X() * vertexbuff.X() + vertexbuff.Y() * vertexbuff.Y());

            if (vertexbuff.Z() > 0 && vertexbuff.Z() < 1000 && radius < 25.0) {

               track->SetTrackVertex(XYZPoint(vertexbuff));
               track_f->SetTrackVertex(XYZPoint(vertexbuff));
               fVertex_1.SetXYZ(vertexbuff.X(), vertexbuff.Y(), vertexbuff.Z());
               fVertex_2.SetXYZ(vertexbuff.X(), vertexbuff.Y(), vertexbuff.Z());
               fVertex_mean = vertexbuff;

               if (!IsFilled[track->GetTrackID()]) {
                  // std::cout<<" Add track  "<<track->GetTrackID()<<std::endl;
                  IsFilled[track->GetTrackID()] = kTRUE;
                  fTrackCand.push_back(*track);
               }

               if (!IsFilled[track_f->GetTrackID()]) {
                  // std::cout<<" Add trackf  "<<track_f->GetTrackID()<<std::endl;
                  IsFilled[track_f->GetTrackID()] = kTRUE;
                  fTrackCand.push_back(*track_f);
               }

               // condition for almost parallel tracks and vertex out of the chamber
            } else if (angle < 10 && angle > 170) {

               TVector3 vertexbuff1 = ClosestPoint2Lines(e1, p1, BeamDir, BeamPoint);
               TVector3 vertexbuff2 = ClosestPoint2Lines(e2, p2, BeamDir, BeamPoint);
               TVector3 vertexbuffav = 0.5 * (vertexbuff1 + vertexbuff2);
               TVector3 vertexbuffdif = vertexbuff1 - vertexbuff2;

               if (vertexbuffdif.Mag() > 2 * fLineDistThreshold)
                  continue;

               track->SetTrackVertex(XYZPoint(vertexbuffav));
               track_f->SetTrackVertex(XYZPoint(vertexbuffav));
               fVertex_1.SetXYZ(vertexbuffav.X(), vertexbuffav.Y(), vertexbuffav.Z());
               fVertex_2.SetXYZ(vertexbuffav.X(), vertexbuffav.Y(), vertexbuffav.Z());
               fVertex_mean = vertexbuffav;

               if (!IsFilled[track->GetTrackID()]) {
                  // std::cout<<" Add track  "<<track->GetTrackID()<<std::endl;
                  IsFilled[track->GetTrackID()] = kTRUE;
                  fTrackCand.push_back(*track);
               }

               if (!IsFilled[track_f->GetTrackID()]) {
                  // std::cout<<" Add trackf  "<<track_f->GetTrackID()<<std::endl;
                  IsFilled[track_f->GetTrackID()] = kTRUE;
                  fTrackCand.push_back(*track_f);
               }
            }

         } // if fLineDistThreshold

      } // End of track_f

   } // Loop over the tracks

   if (fTrackCand.size() > 5)
      fTrackCand.resize(5); // Truncate the maximum number of lines to 5
}

void AtRansacMod::FindVertexOneTrack(std::vector<AtTrack *> tracks)
{

   // Assumes the minimum distance between two lines, with respect a given threshold, the first vertex candidate. Then
   // evaluates the distance of each remaining line with respect to the others (vertex) to decide the particles of the
   // reaction.
   // std::cout<<" New find vertex call "<<std::endl;

   Double_t mad = 10; // Minimum approach distance. This is the minimum distance between the lines in 3D. Must be bigger
                      // than fLineDistThreshold
   // ROOT::Math::XYZVector c_1(-1000,-1000,-1000);
   // ROOT::Math::XYZVector c_2(-1000,-1000,-1000);
   TVector3 c_1(-1000, -1000, -1000);
   TVector3 c_2(-1000, -1000, -1000);
   // std::vector<AtTrack*> *TrackCand;

   // Current  parametrization
   // x = p[0] + p[1]*t;
   // y = p[2] + p[3]*t;
   // z = p[4] + p[5]*t;
   //  (x,y,z) = (p[0],p[2],p[4]) + (p[1],p[3],p[5])*t

   // Vector of the beam determined from the experimental data
   TVector3 BeamDir(0., 0., 1.0);
   TVector3 BeamPoint(0., 0., 500.0);

   std::vector<Bool_t> IsFilled;
   for (Int_t i = 0; i < int(tracks.size()); i++)
      IsFilled.push_back(kFALSE);

   // Test each line against the others to find a vertex candidate
   for (Int_t i = 0; i < int(tracks.size()); i++) {

      AtTrack *track = tracks.at(i);
      std::vector<Double_t> p = track->GetFitPar();

      if (p.size() == 0)
         continue;

      TVector3 p1(p[0], p[2], p[4]); // p1
      TVector3 e1(p[1], p[3], p[5]); // d1

      double angle = e1.Angle(BeamDir) * 180. / 3.1415;

      TVector3 n = e1.Cross(BeamDir);
      double sdist = fabs(n.Dot(p1 - BeamPoint) / n.Mag());
      TVector3 vertexbuff = ClosestPoint2Lines(e1, p1, BeamDir, BeamPoint);

      if (sdist < fLineDistThreshold) {
         TVector3 meanVer = vertexbuff;
         double radius = sqrt(vertexbuff.X() * vertexbuff.X() + vertexbuff.Y() * vertexbuff.Y());

         // if(vertexbuff.Z()>0 && vertexbuff.Z()<1000 && radius<25.0){
         if (radius < 25.0) {
            track->SetTrackVertex(XYZPoint(vertexbuff));
            fVertex_1.SetXYZ(vertexbuff.X(), vertexbuff.Y(), vertexbuff.Z());
            fVertex_mean = vertexbuff;

            if (!IsFilled[track->GetTrackID()]) {
               // std::cout<<" Add track  "<<track->GetTrackID()<<std::endl;
               IsFilled[track->GetTrackID()] = kTRUE;
               fTrackCand.push_back(*track);
            }

            // condition for almost parallel tracks and vertex out of the chamber
         }

      } // if fLineDistThreshold

   } // Loop over the tracks

   if (fTrackCand.size() > 5)
      fTrackCand.resize(5); // Truncate the maximum number of lines to 5
}

TVector3 AtRansacMod::ClosestPoint2Lines(TVector3 d1, TVector3 pt1, TVector3 d2, TVector3 pt2)
{
   TVector3 n1 = d1.Cross(d2.Cross(d1));
   TVector3 n2 = d2.Cross(d1.Cross(d2));
   double t1 = (pt2 - pt1).Dot(n2) / (d1.Dot(n2));
   double t2 = (pt1 - pt2).Dot(n1) / (d2.Dot(n1));
   TVector3 c1 = pt1 + t1 * d1;
   TVector3 c2 = pt2 + t2 * d2;
   TVector3 meanpoint = 0.5 * (c1 + c2);

   return meanpoint;
}

Double_t AtRansacMod::Fit3D(vector<int> inliners, TVector3 &V1, TVector3 &V2)
{

   //------3D Line Regression
   //----- adapted from: http://fr.scribd.com/doc/31477970/Regressions-et-trajectoires-3D
   int R, C;
   double Q;
   double Xm, Ym, Zm;
   double Xh, Yh, Zh;
   double a, b;
   double Sxx, Sxy, Syy, Sxz, Szz, Syz;
   double theta;
   double K11, K22, K12, K10, K01, K00;
   double c0, c1, c2;
   double p, q, r, dm2;
   double rho, phi;

   Q = Xm = Ym = Zm = 0.;
   double total_charge = 0;
   Sxx = Syy = Szz = Sxy = Sxz = Syz = 0.;

   for (auto i : inliners) {
      Q += vQ[i] / 10.;
      Xm += vX[i] * vQ[i] / 10.;
      Ym += vY[i] * vQ[i] / 10.;
      Zm += vZ[i] * vQ[i] / 10.;
      Sxx += vX[i] * vX[i] * vQ[i] / 10.;
      Syy += vY[i] * vY[i] * vQ[i] / 10.;
      Szz += vZ[i] * vZ[i] * vQ[i] / 10.;
      Sxy += vX[i] * vY[i] * vQ[i] / 10.;
      Sxz += vX[i] * vZ[i] * vQ[i] / 10.;
      Syz += vY[i] * vZ[i] * vQ[i] / 10.;
   }
   // vTrackCharge.push_back(total_charge);

   Xm /= Q;
   Ym /= Q;
   Zm /= Q;
   Sxx /= Q;
   Syy /= Q;
   Szz /= Q;
   Sxy /= Q;
   Sxz /= Q;
   Syz /= Q;
   Sxx -= (Xm * Xm);
   Syy -= (Ym * Ym);
   Szz -= (Zm * Zm);
   Sxy -= (Xm * Ym);
   Sxz -= (Xm * Zm);
   Syz -= (Ym * Zm);

   theta = 0.5 * atan((2. * Sxy) / (Sxx - Syy));

   K11 = (Syy + Szz) * pow(cos(theta), 2) + (Sxx + Szz) * pow(sin(theta), 2) - 2. * Sxy * cos(theta) * sin(theta);
   K22 = (Syy + Szz) * pow(sin(theta), 2) + (Sxx + Szz) * pow(cos(theta), 2) + 2. * Sxy * cos(theta) * sin(theta);
   K12 = -Sxy * (pow(cos(theta), 2) - pow(sin(theta), 2)) + (Sxx - Syy) * cos(theta) * sin(theta);
   K10 = Sxz * cos(theta) + Syz * sin(theta);
   K01 = -Sxz * sin(theta) + Syz * cos(theta);
   K00 = Sxx + Syy;

   c2 = -K00 - K11 - K22;
   c1 = K00 * K11 + K00 * K22 + K11 * K22 - K01 * K01 - K10 * K10;
   c0 = K01 * K01 * K11 + K10 * K10 * K22 - K00 * K11 * K22;

   p = c1 - pow(c2, 2) / 3.;
   q = 2. * pow(c2, 3) / 27. - c1 * c2 / 3. + c0;
   r = pow(q / 2., 2) + pow(p, 3) / 27.;

   if (r > 0)
      dm2 = -c2 / 3. + pow(-q / 2. + sqrt(r), 1. / 3.) + pow(-q / 2. - sqrt(r), 1. / 3.);
   if (r < 0) {
      rho = sqrt(-pow(p, 3) / 27.);
      phi = acos(-q / (2. * rho));
      dm2 = min(-c2 / 3. + 2. * pow(rho, 1. / 3.) * cos(phi / 3.),
                min(-c2 / 3. + 2. * pow(rho, 1. / 3.) * cos((phi + 2. * TMath::Pi()) / 3.),
                    -c2 / 3. + 2. * pow(rho, 1. / 3.) * cos((phi + 4. * TMath::Pi()) / 3.)));
   }

   a = -K10 * cos(theta) / (K11 - dm2) + K01 * sin(theta) / (K22 - dm2);
   b = -K10 * sin(theta) / (K11 - dm2) - K01 * cos(theta) / (K22 - dm2);

   Xh = ((1. + b * b) * Xm - a * b * Ym + a * Zm) / (1. + a * a + b * b);
   Yh = ((1. + a * a) * Ym - a * b * Xm + b * Zm) / (1. + a * a + b * b);
   Zh = ((a * a + b * b) * Zm + a * Xm + b * Ym) / (1. + a * a + b * b);

   V1.SetXYZ(Xm, Ym, Zm);
   V2.SetXYZ(Xh, Yh, Zh);

   return (fabs(dm2 / Q));
}
