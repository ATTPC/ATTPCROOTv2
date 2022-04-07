#include "AtRansac.h"

// FairRoot classes
#include <FairRuntimeDb.h>
#include <FairRun.h>

using namespace ROOT::Math;

ClassImp(AtRANSACN::AtRansac);

AtRANSACN::AtRansac::AtRansac()
{
   fVertex_1.SetXYZ(-10000, -10000, -10000);
   fVertex_2.SetXYZ(-10000, -10000, -10000);
   fMinimum = -1.0;
   fLineDistThreshold = 3.0;

   fRANSACModel = pcl::SACMODEL_LINE;
   fRANSACThreshold = 5.0;
   fMinHitsLine = 5;

   fXCenter = 0.0;
   fYCenter = 0.0;

   fRANSACPointThreshold = 0.01; // Default 10%

   fVertexTime = -1000.0;

   fTiltAng = 0; // fPar->GetTiltAngle();

   pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

AtRANSACN::AtRansac::~AtRansac() {}

TVector3 AtRANSACN::AtRansac::GetVertex1()
{
   return fVertex_1;
}
TVector3 AtRANSACN::AtRansac::GetVertex2()
{
   return fVertex_2;
}
Double_t AtRANSACN::AtRansac::GetMinimum()
{
   return fMinimum;
}
std::vector<AtTrack> &AtRANSACN::AtRansac::GetTrackCand()
{
   return fTrackCand;
}
std::vector<AtRANSACN::AtRansac::PairedLines> AtRANSACN::AtRansac::GetPairedLinesArray()
{
   return PLines;
}
std::pair<Int_t, Int_t> AtRANSACN::AtRansac::GetPairTracksIndex()
{
   return fVertex_tracks;
}
Double_t AtRANSACN::AtRansac::GetVertexTime()
{
   return fVertexTime;
}
TVector3 AtRANSACN::AtRansac::GetVertexMean()
{
   return fVertex_mean;
}

void AtRANSACN::AtRansac::SetTiltAngle(Double_t val)
{
   fTiltAng = val;
}
void AtRANSACN::AtRansac::SetModelType(int model)
{
   fRANSACModel = model;
}
void AtRANSACN::AtRansac::SetDistanceThreshold(Float_t threshold)
{
   fRANSACThreshold = threshold;
}
void AtRANSACN::AtRansac::SetMinHitsLine(Int_t nhits)
{
   fMinHitsLine = nhits;
}
void AtRANSACN::AtRansac::SetXYCenter(Double_t xc, Double_t yc)
{
   fXCenter = xc;
   fYCenter = yc;
}
void AtRANSACN::AtRansac::SetRANSACPointThreshold(Float_t val)
{
   fRANSACPointThreshold = val;
}
void AtRANSACN::AtRansac::SetVertexTime(Double_t val)
{
   fVertexTime = val;
}

void AtRANSACN::AtRansac::CalcRANSAC(AtEvent *event)
{

   std::vector<AtTrack> *tracks = RansacPCL(event->GetHitArray());

   Int_t tracksSize = tracks->size();
   std::cout << "RansacPCL tracks size : " << tracksSize << std::endl;
   if (tracksSize > 1) {
      for (Int_t ntrack = 0; ntrack < tracksSize; ntrack++) {
         std::vector<AtHit> trackHits = tracks->at(ntrack).GetHitArray();
         Int_t nHits = trackHits.size();
         // std::cout<<" Num  Hits : "<<nHits<<std::endl;
         if (nHits > 5) {
            // MinimizeTrack(tracks.at(ntrack));
            double mychi2 = Fit3D(&tracks->at(ntrack));
            tracks->at(ntrack).SetTrackID(ntrack);
            fTrackCand.push_back(tracks->at(ntrack));
         }
      } // Tracks loop
      // FindVertex(tracks);

   } // Minimum tracks
}

void AtRANSACN::AtRansac::CalcRANSACFull(AtEvent *event)
{

   std::vector<AtTrack> *tracks = RansacPCL(event->GetHitArray());

   XYZVector Z_1(0.0, 0.0, 1.0); // Beam direction

   Int_t tracksSize = tracks->size();
   std::cout << "RansacPCL tracks size : " << tracksSize << std::endl;
   if (tracksSize > 1) { // Defined in CalcGenHoughSpace
      for (Int_t ntrack = 0; ntrack < tracksSize; ntrack++) {
         std::vector<AtHit> trackHits = tracks->at(ntrack).GetHitArray();
         Int_t nHits = trackHits.size();

         if (nHits > fMinHitsLine) // We only accept lines with more than 5 hits and a maximum number of lines of 5
         {
            MinimizeTrack(&tracks->at(ntrack));
            tracks->at(ntrack).SetTrackID(ntrack);
            std::vector<Double_t> p = tracks->at(ntrack).GetFitPar();
            if (p.size() == 4) {
               XYZVector L_1(p[1], p[3], 1.);
               Double_t angZDeti = GetAngleTracks(L_1, Z_1);
               tracks->at(ntrack).SetAngleZAxis(angZDeti);
               fTrackCand.push_back(tracks->at(ntrack));
            }
         } else
            LOG(error) << "Rejecting track: " << ntrack << " only " << nHits << " require " << fMinHitsLine;

      } // Tracks loop
      if (fTrackCand.size() > 5)
         fTrackCand.resize(5);
   } // Minimum tracks

   // FindVertex(tracks);
}

std::vector<AtTrack> *AtRANSACN::AtRansac::RansacPCL(const std::vector<AtHit> &hits)
{
   LOG(debug) << "Running ransac with PCL";

   // initialize PointClouds
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGBA>);

   // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);

   if (hits.size() < 5)
      return &fRansacTracks;

   Int_t nHits = hits.size();
   cloud->points.resize(nHits);

   LOG(debug) << "Filling cloud point";

   for (int hitIndex = 0; hitIndex < hits.size(); ++hitIndex) {

      const auto hit = hits[hitIndex];
      auto position = hit.GetPosition();
      auto hitID = hit.GetHitID();

      cloud->points[hitID].x = position.X();
      cloud->points[hitID].y = position.Y();
      cloud->points[hitID].z = position.Z();
      cloud->points[hitID].rgb = hitIndex; // Storing the position of the hit in the event container
   }

   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
   seg.setOptimizeCoefficients(true);
   seg.setModelType(fRANSACModel); // https://pointclouds.org/documentation/group__sample__consensus.html
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setMaxIterations(1000);
   seg.setDistanceThreshold(fRANSACThreshold);

   // Create the filtering object
   pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

   int i = 0, nr_points = (int)cloud->points.size();

   while (cloud->points.size() > fRANSACPointThreshold * nr_points) {
      // Segment the largest planar component from the remaining cloud
      // std::cout<<cloud->points.size()<<std::endl;

      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);

      std::vector<Double_t> coeff;

      // size of vector coefficients->values is not always 6!
      int Coefsize = coefficients->values.size();

      for (auto icoeff = 0; icoeff < Coefsize; ++icoeff)
         coeff.push_back(coefficients->values[icoeff]);

      if (inliers->indices.size() == 0)
         break;

      // Extract the inliers
      extract.setInputCloud(cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*cloud_p);

      if (cloud_p->points.size() > 0) {
         AtTrack track;

         LOG(debug) << "Filling a track with hits";
         for (Int_t iHit = 0; iHit < cloud_p->points.size(); iHit++) {

            /*if (&hits.at(cloud_p->points[iHit].rgb))
               track.AddHit(&hits.at(cloud_p->points[iHit].rgb));
       */
            LOG(debug2) << "Getting hit index: " << cloud_p->points[iHit].rgb << " from hits";
            track.AddHit(hits.at(cloud_p->points[iHit].rgb));
         }

         track.SetRANSACCoeff(coeff);

         fRansacTracks.push_back(track);

         // delete track;
      }
      // std::stringstream ss;
      // ss << "../track_" << i << ".pcd";
      // writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

      // Create the filtering object
      extract.setNegative(true);
      extract.filter(*cloud_f);
      cloud.swap(cloud_f);
      i++;
   }

   LOG(debug) << "Finishing ransac with PCL";
   return &fRansacTracks;
}

Int_t AtRANSACN::AtRansac::MinimizeTrack(AtTrack *track)
{

   gErrorIgnoreLevel = kFatal;
   Int_t nd = 10000;
   TGraph2D *gr = new TGraph2D(); /////NB: This should be created on the heap only once so it should move outside of
                                  /// this function!!!!!!!!!!!!!!!
   std::vector<AtHit> HitArray = track->GetHitArray();

   double p0[4] = {10, 20, 1, 2}; // For the moment those are dummy parameters

   for (int hitIndex = 0; hitIndex < HitArray.size(); hitIndex++) {
      auto pos = HitArray[hitIndex].GetPosition();
      gr->SetPoint(hitIndex, pos.X(), pos.Y(), pos.Z());
   }

   ROOT::Fit::Fitter fitter;
   SumDistance2 sdist(gr);
#ifdef __CINT__
   ROOT::Math::Functor fcn(&sdist, 4, "SumDistance2");
#else
   ROOT::Math::Functor fcn(sdist, 4);
#endif
   // set the function and the initial parameter values
   double pStart[4] = {1, 1, 1, 1};
   fitter.SetFCN(fcn, pStart);
   // set step sizes different than default ones (0.3 times parameter values)
   for (int i = 0; i < 4; ++i)
      fitter.Config().ParSettings(i).SetStepSize(0.01);

   bool ok = fitter.FitFCN();
   if (!ok) {
      Error("line3Dfit", "Line3D Fit failed");
      return 1;
   }

   const ROOT::Fit::FitResult &result = fitter.Result();
   const ROOT::Math::Minimizer *min = fitter.GetMinimizer();
   double sigma2 = 25.0; // Size of the pad
   double Chi2_min = min->MinValue();
   int NDF = min->NFree();
   int npoints = gr->GetN();
   const double *parFitBuff = result.GetParams();
   std::vector<Double_t> parFit;
   for (Int_t i = 0; i < 4; i++)
      parFit.push_back(parFitBuff[i]); // 4 parameters per fit
   track->SetFitPar(parFit);
   track->SetMinimum(Chi2_min);
   track->SetNFree(NDF);

   std::cout << parFit[0] << " " << parFit[1] << "  " << parFit[2] << " " << parFit[3] << std::endl;
   std::cout << " Chi2 (Minuit) : " << Chi2_min << " NDF : " << NDF << std::endl;
   std::cout << " Chi2 reduced  : " << (Chi2_min / sigma2 / (double)npoints) << std::endl;

   // std::cout << "Total final distance square " << result.MinFcnValue() << std::endl;
   // result.Print(std::cout);

   // Draw the fit
   /*gr->Draw("p0");
   const double * parFit = result.GetParams();
   int n = 1000;
   double t0 = 0;
   double dt = 1000;
   TPolyLine3D *l = new TPolyLine3D(n);
   for (int i = 0; i <n;++i) {
      double t = t0+ dt*i/n;
      double x,y,z;
      SetLine(t,parFit,x,y,z);
      l->SetPoint(i,x,y,z);
      //std::cout<<" x : "<<x<<" y : "<<y<<"  z : "<<z<<std::endl;
   }
   l->SetLineColor(kRed);
   l->Draw("same");*/

   return 0;
}

void AtRANSACN::AtRansac::FindVertex(std::vector<AtTrack *> tracks)
{

   // Assumes the minimum distance between two lines, with respect a given threshold, the first vertex candidate. Then
   // evaluates the distance of each remaining line with respect to the others (vertex) to decide the particles of the
   // reaction.
   // std::cout<<" New find vertex call "<<std::endl;

   Double_t mad = 10; // Minimum approach distance. This is the minimum distance between the lines in 3D. Must be bigger
                      // than fLineDistThreshold
   XYZVector c_1(-1000, -1000, -1000);
   XYZVector c_2(-1000, -1000, -1000);
   // std::vector<AtTrack*> *TrackCand;

   // Current  parametrization
   // x = p[0] + p[1]*t;
   // y = p[2] + p[3]*t;
   // z = t;
   //  (x,y,z) = (p[0],p[2],0) + (p[1],p[3],1)*t

   // Equation of the Z axis in the solenoid frame
   XYZVector Z_0(0, 0, 1000.0);
   XYZVector Z_1(0, 0, 1);

   // Equation of the Y axis in the
   XYZVector Y_0(0, 0, 1000.0);
   XYZVector Y_1(0, 1, 0);

   // Counter clockwise rotations
   // Direction of the Z and Y axis in the detector frame (This is the rotated detector), this is used to calculater the
   // theta and phi angles
   RotationX rx(-fTiltAng * TMath::Pi() / 180.0);
   XYZVector Z_1_rot(rx * Z_1);

   RotationX ry(-fTiltAng * TMath::Pi() / 180.0);
   XYZVector Y_1_rot(ry * Y_1);

   // Vector of the beam determined from the experimental data
   // XYZVector BeamDir_1(-0.106359,-0.0348344,1.0);
   XYZVector BeamDir_1(0., 0., 1.0);
   // TODO:: This is for 6.5 degrees of tilting angle. Need a function to set it.

   // Test each line against the others to find a vertex candidate
   for (Int_t i = 0; i < int(tracks.size()) - 1; i++) {

      AtTrack *track = tracks.at(i);
      track->SetTrackID(i);
      std::vector<Double_t> p = track->GetFitPar();

      if (p.size() > 0) {
         // XYZVector L_0(p[0], p[2], 0. );//p1
         // XYZVector L_1(p[1], p[3], 1. );//d1
         XYZVector L_0(p[0], p[2], p[4]); // p1
         XYZVector L_1(p[1], p[3], p[5]); // d1

         // std::cout<<" L_1 p[0] : "<<p[0]<<" L_1 p[2] : "<<p[2]<<std::endl;
         // std::cout<<" L_1 p[1] : "<<p[1]<<" L_1 p[3] : "<<p[3]<<std::endl;

         for (Int_t j = i + 1; j < tracks.size(); j++) {
            AtTrack *track_f = tracks.at(j);
            track_f->SetTrackID(j);
            std::vector<Double_t> p_f = track_f->GetFitPar();

            if (p_f.size() > 0) {
               // XYZVector L_f0(p_f[0], p_f[2], 0. );//p2
               // XYZVector L_f1(p_f[1], p_f[3], 1. );//d2
               XYZVector L_f0(p_f[0], p_f[2], p_f[4]); // p2
               XYZVector L_f1(p_f[1], p_f[3], p_f[5]); // d2

               // std::cout<<" L_f0 p_f[0] : "<<p_f[0]<<" L_f1 p_f[2] : "<<p_f[2]<<std::endl;
               // std::cout<<" L_f1 p_f[1] : "<<p_f[1]<<" L_f1 p_f[3] : "<<p_f[3]<<std::endl;

               XYZVector L = L_1.Cross(L_f1);
               Double_t L_mag = L.Rho();
               XYZVector n_1 = L_1.Cross(L);
               XYZVector n_2 = L_f1.Cross(L);
               c_1 = L_0 + ((L_f0 - L_0).Dot(n_2) * L_1) / (L_1.Dot(n_2));
               c_2 = L_f0 + ((L_0 - L_f0).Dot(n_1) * L_f1) / (L_f1.Dot(n_1));

               // std::cout<<i<<" "<<j<<" "<<L_mag<<std::endl;

               if (L_mag > 0) {
                  XYZVector n = L / (Double_t)L_mag;
                  Double_t d = TMath::Abs(n.Dot(L_0 - L_f0));
                  // std::cout<<" Distance of minimum approach : "<<d<<std::endl;
                  Double_t num = L_1.X() * L_f1.X() + L_1.Y() * L_f1.Y() + L_1.Z() * L_f1.Z();
                  Double_t den = TMath::Sqrt(L_1.X() * L_1.X() + L_1.Y() * L_1.Y() + L_1.Z() * L_1.Z()) *
                                 TMath::Sqrt(L_f1.X() * L_f1.X() + L_f1.Y() * L_f1.Y() + L_f1.Z() * L_f1.Z());
                  // NB:: The vector of hits is always sorted in time so the direction of porpagation of the particle is
                  // always positive. This means that the angle will be always between 0 and 90 degree. The vertex and
                  // the mean time of the track are needed to determine the direction of the track and then add 90
                  // degrees.
                  Double_t ang2 = TMath::ACos(num / den);
                  XYZVector vertex1_buff;
                  vertex1_buff.SetXYZ(c_1.X(), c_1.Y(), c_1.Z());
                  XYZVector vertex2_buff;
                  vertex2_buff.SetXYZ(c_2.X(), c_2.Y(), c_2.Z());

                  // Angle with respect to solenoid
                  Double_t angZi = GetAngleTracks(L_1, Z_1);
                  Double_t angZj = GetAngleTracks(L_f1, Z_1);

                  // Angles with respect to tilted detector
                  Double_t angZDeti = GetAngleTracks(L_1, BeamDir_1);
                  Double_t angZDetj = GetAngleTracks(L_f1, BeamDir_1);
                  Double_t angYDeti = 0.0; // GetAngleTracks(L_1,Y_1_rot);
                  Double_t angYDetj = 0.0; // GetAngleTracks(L_f1,Y_1_rot);

                  track->SetAngleZAxis(angZi);
                  track->SetAngleZDet(angZDeti);
                  track->SetAngleYDet(angYDeti);

                  track_f->SetAngleZAxis(angZj);
                  track_f->SetAngleZDet(angZDetj);
                  track_f->SetAngleYDet(angYDetj);

                  XYZPoint vertexAvg(0.5 * (vertex1_buff + vertex2_buff));
                  track->SetTrackVertex(vertexAvg);
                  track_f->SetTrackVertex(vertexAvg);

                  if (d < mad) {

                     mad = d;
                     // std::cout<<" New distance of minimum approach : "<<mad<<std::endl;
                     // std::cout<<" Angle between lines i : "<<i<<" j : "<<j<<"  "<<ang2<<std::endl;

                     // Global event variables
                     fVertex_1.SetXYZ(c_1.X(), c_1.Y(), c_1.Z());
                     fVertex_2.SetXYZ(c_2.X(), c_2.Y(), c_2.Z());
                     fVertex_mean = 0.5 * (fVertex_1 + fVertex_2);
                     fVertex_tracks.first = i;
                     fVertex_tracks.second = j;
                     fMinimum = mad;

                     if (!CheckTrackID(track->GetTrackID(), &fTrackCand)) {
                        fTrackCand.push_back(*track);
                        PairedLines PL;
                        PL.LinesID.first = i;
                        PL.LinesID.second = j;
                        PL.AngleZAxis.first = angZi;
                        PL.AngleZAxis.second = angZj;
                        PL.AngleZDet.first = angZDeti;
                        PL.AngleZDet.second = angZDetj;
                        PL.AngleYDet.first = angYDeti;
                        PL.AngleYDet.second = angYDetj;
                        PL.minDist = mad;
                        PL.meanVertex = fVertex_mean;
                        PL.angle = ang2;
                        PLines.push_back(PL);
                     }
                     if (!CheckTrackID(track_f->GetTrackID(), &fTrackCand)) {
                        fTrackCand.push_back(*track_f);
                        PairedLines PL;
                        PL.LinesID.first = i;
                        PL.LinesID.second = j;
                        PL.AngleZAxis.first = angZi;
                        PL.AngleZAxis.second = angZj;
                        PL.AngleZDet.first = angZDeti;
                        PL.AngleZDet.second = angZDetj;
                        PL.AngleYDet.first = angYDeti;
                        PL.AngleYDet.second = angYDetj;
                        PL.minDist = mad;
                        PL.meanVertex = fVertex_mean;
                        PL.angle = ang2;
                        PLines.push_back(PL);
                     }
                  }

                  if (d < fLineDistThreshold) {

                     if (!CheckTrackID(track->GetTrackID(), &fTrackCand)) {
                        // std::cout<<" Add track"<<track->GetTrackID()<<std::endl;
                        fTrackCand.push_back(*track);
                        PairedLines PL;
                        PL.LinesID.first = i;
                        PL.LinesID.second = j;
                        PL.AngleZAxis.first = angZi;
                        PL.AngleZAxis.second = angZj;
                        PL.AngleZDet.first = angZDeti;
                        PL.AngleZDet.second = angZDetj;
                        PL.AngleYDet.first = angYDeti;
                        PL.AngleYDet.second = angYDetj;
                        PL.minDist = d;
                        PL.meanVertex = TVector3(vertexAvg.X(), vertexAvg.Y(), vertexAvg.Z());
                        PL.angle = ang2;
                        PLines.push_back(PL);
                     }

                     if (!CheckTrackID(track_f->GetTrackID(), &fTrackCand)) {
                        // std::cout<<" Add track f"<<track_f->GetTrackID()<<std::endl;
                        fTrackCand.push_back(*track_f);
                        PairedLines PL;
                        PL.LinesID.first = i;
                        PL.LinesID.second = j;
                        PL.AngleZAxis.first = angZi;
                        PL.AngleZAxis.second = angZj;
                        PL.AngleZDet.first = angZDeti;
                        PL.AngleZDet.second = angZDetj;
                        PL.AngleYDet.first = angYDeti;
                        PL.AngleYDet.second = angYDetj;
                        PL.minDist = d;
                        PL.meanVertex = TVector3(vertexAvg.X(), vertexAvg.Y(), vertexAvg.Z());
                        PL.angle = ang2;
                        PLines.push_back(PL);
                     }
                  }
               }

            } // p_f size
         }    // End of track
      }       // p size

   } // Loop over the tracks

   if (fTrackCand.size() > 5)
      fTrackCand.resize(5); // Truncate the maximum number of lines to 5
}

Double_t AtRANSACN::AtRansac::distance2(double x, double y, double z, const double *p)
{

   // distance line point is D= | (xp-x0) cross  ux |
   // where ux is direction of line and x0 is a point in the line (like t = 0) and x1 is in t=1
   XYZVector xp(x, y, z);
   XYZVector x0(p[0], p[2], 0.);
   XYZVector x1(p[0] + p[1], p[2] + p[3], 1.);
   XYZVector u = (x1 - x0).Unit();
   double d2 = ((xp - x0).Cross(u)).Mag2();
   return d2;
}

void AtRANSACN::AtRansac::SetLine(double t, const double *p, double &x, double &y, double &z)
{
   // a parameteric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = p[0] + p[1] * t;
   y = p[2] + p[3] * t;
   z = t;
}

Bool_t AtRANSACN::AtRansac::CheckTrackID(Int_t trackID, std::vector<AtTrack> *trackArray)
{
   auto it = find_if(trackArray->begin(), trackArray->end(),
                     [&trackID](AtTrack &track) { return track.GetTrackID() == trackID; });
   if (it != trackArray->end()) {
      auto hitInd = std::distance<std::vector<AtTrack>::const_iterator>(trackArray->begin(), it);
      return kTRUE;
   } else
      return kFALSE;
}

Double_t AtRANSACN::AtRansac::GetAngleTracks(const ROOT::Math::XYZVector &vec1, const ROOT::Math::XYZVector &vec2)
{

   // NB:: The vector of hits is always sorted in time so the direction of porpagation of the particle is always
   // positive. This means that the angle will be always between 0 and 90 degree. The vertex and the mean time of the
   // track are needed to determine the direction of the track and then add 90 degrees.
   Double_t num = vec1.X() * vec2.X() + vec1.Y() * vec2.Y() + vec1.Z() * vec2.Z();
   Double_t den = TMath::Sqrt(vec1.X() * vec1.X() + vec1.Y() * vec1.Y() + vec1.Z() * vec1.Z()) *
                  TMath::Sqrt(vec2.X() * vec2.X() + vec2.Y() * vec2.Y() + vec2.Z() * vec2.Z());
   Double_t ang = TMath::ACos(num / den);

   return ang;
}

Int_t AtRANSACN::AtRansac::FindIndexTrack(Int_t index)
{
   auto it =
      find_if(fTrackCand.begin(), fTrackCand.end(), [&index](AtTrack &track) { return track.GetTrackID() == index; });
   if (it != fTrackCand.end()) {
      auto trackInd = std::distance<std::vector<AtTrack>::const_iterator>(fTrackCand.begin(), it);
      return trackInd;
   } else
      return -1;
}

Double_t AtRANSACN::AtRansac::Fit3D(AtTrack *track)
{

   //--------Adapted from https://www.scribd.com/doc/31477970/Regressions-et-trajectoires-3D
   //--------3D linear regression

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
   Sxx = Syy = Szz = Sxy = Sxz = Syz = 0.;

   std::vector<AtHit> HitArray = track->GetHitArray();
   std::vector<double> X;
   std::vector<double> Y;
   std::vector<double> Z;
   std::vector<double> Charge;

   for (Int_t i = 0; i < HitArray.size(); i++) {
      AtHit hit = HitArray.at(i);
      auto pos = hit.GetPosition();
      Double_t tq = hit.GetCharge();
      X.push_back(pos.X());
      Y.push_back(pos.Y());
      Z.push_back(pos.Z());
      Charge.push_back(q);
   }

   for (Int_t i = 0; i < HitArray.size(); i++) {
      Q += Charge[i] / 10.;
      Xm += X[i] * Charge[i] / 10.;
      Ym += Y[i] * Charge[i] / 10.;
      Zm += Z[i] * Charge[i] / 10.;
      Sxx += X[i] * X[i] * Charge[i] / 10.;
      Syy += Y[i] * Y[i] * Charge[i] / 10.;
      Szz += Z[i] * Z[i] * Charge[i] / 10.;
      Sxy += X[i] * Y[i] * Charge[i] / 10.;
      Sxz += X[i] * Z[i] * Charge[i] / 10.;
      Syz += Y[i] * Z[i] * Charge[i] / 10.;
   }

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
      dm2 = std::min(-c2 / 3. + 2. * pow(rho, 1. / 3.) * cos(phi / 3.),
                     std::min(-c2 / 3. + 2. * pow(rho, 1. / 3.) * cos((phi + 2. * TMath::Pi()) / 3.),
                              -c2 / 3. + 2. * pow(rho, 1. / 3.) * cos((phi + 4. * TMath::Pi()) / 3.)));
   }

   a = -K10 * cos(theta) / (K11 - dm2) + K01 * sin(theta) / (K22 - dm2);
   b = -K10 * sin(theta) / (K11 - dm2) - K01 * cos(theta) / (K22 - dm2);

   Xh = ((1. + b * b) * Xm - a * b * Ym + a * Zm) / (1. + a * a + b * b);
   Yh = ((1. + a * a) * Ym - a * b * Xm + b * Zm) / (1. + a * a + b * b);
   Zh = ((a * a + b * b) * Zm + a * Xm + b * Ym) / (1. + a * a + b * b);

   std::vector<Double_t> par;
   par.push_back(Xm);      // 0
   par.push_back(Xh - Xm); // 1
   par.push_back(Ym);      // 2
   par.push_back(Yh - Ym); // 3
   par.push_back(Zm);      // 4
   par.push_back(Zh - Zm); // 5

   track->SetFitPar(par);
   track->SetMinimum(fabs(dm2 / Q));
   track->SetNFree(X.size() - 6);
   X.clear();
   Y.clear();
   Z.clear();
   Charge.clear();

   return (fabs(dm2 / Q));
}
