#include "AtPRA.h"

#include "AtContainerManip.h"
#include "AtHit.h"     // for AtHit, XYZPoint
#include "AtPattern.h" // for AtPattern
#include "AtPatternCircle2D.h"
#include "AtPatternEvent.h"
#include "AtPatternLine.h"
#include "AtPatternTypes.h" // for PatternType, PatternTy...
#include "AtSampleConsensus.h"
#include "AtTrack.h" // for XYZPoint, AtTrack

#include <FairLogger.h>

#include <Math/Point3D.h>     // for PositionVector3D, Cart...
#include <Math/Vector2D.h>    // for PositionVector3D, Cart...
#include <Math/Vector2Dfwd.h> // for XYVector
#include <Math/Vector3D.h>    // for DisplacementVector3D
#include <TGraph.h>           // for TGraph
#include <TMath.h>            // for Power, Sqrt, ATan2, Pi

#include <algorithm> // for max, for_each, copy_if
#include <cmath>     // for fabs, acos
#include <cstddef>   // for size_t
#include <exception> // for exception
#include <iostream>  // for operator<<, basic_ostream
#include <memory>    // for shared_ptr, __shared_p...
ClassImp(AtPATTERN::AtPRA);

/**
 * @brief Set initial parameters for HC.
 *
 * In track, sets GeoTheta, GeoPhi, GeoCenter, GeoRadius.
 */
void AtPATTERN::AtPRA::SetTrackInitialParameters(AtTrack &track)
{

   /*
std::cout << " Processing track with " << track.GetHitArray().size() << " points."
             << "\n";
   for (auto &hit : track.GetHitArray())
      std::cout << hit.GetTimeStamp() << " ";
   std::cout << std::endl;
   */

   SampleConsensus::AtSampleConsensus RansacSmoothRadius;
   RansacSmoothRadius.SetPatternType(AtPatterns::PatternType::kCircle2D);
   RansacSmoothRadius.SetMinHitsPattern(0.1 * track.GetHitArray().size());
   RansacSmoothRadius.SetDistanceThreshold(6.0);
   RansacSmoothRadius.SetNumIterations(1000);
   auto circularTracks = RansacSmoothRadius.Solve(ContainerManip::GetConstPointerVector(track.GetHitArray()))
                            .GetTrackCand(); // Only part of the spiral is used
                                             // This function also sets the coefficients
                                             // i.e. radius of curvature and center

   if (!circularTracks.empty()) {

      auto &hits = circularTracks.at(0).GetHitArray();

      // auto circle = dynamic_cast<const AtPatterns::AtPatternCircle2D *>(circularTracks.at(0).GetPattern());

      auto circle = std::make_unique<AtPatterns::AtPatternCircle2D>();
      circle->AtPattern::FitPattern(ContainerManip::GetConstPointerVector(track.GetHitArray()));

      auto center = circle->GetCenter();
      auto radius = circle->GetRadius();

      track.SetGeoCenter({center.X(), center.Y()});
      track.SetGeoRadius(radius);
      track.SetPattern(circle->Clone());

      circularTracks.at(0).SetPattern(std::move(circle));

      // std::vector<double> wpca;
      std::vector<double> whit;
      std::vector<double> arclength;

      auto arclengthGraph = std::make_unique<TGraph>();

      auto posPCA = hits.at(0)->GetPosition();
      auto refPosOnCircle = posPCA - center;
      auto refAng = refPosOnCircle.Phi(); // Bounded between (-Pi,Pi]

      std::vector<AtHit> thetaHits;

      // The number of times we have crossed the -Y axis.
      // Increase by 1 when moving from +y to -y
      // Decrease by 1 when moving from -y to +y
      int numYCross = 0;
      int lastYSign = GetSign(refPosOnCircle.Y());

      for (size_t i = 0; i < hits.size(); ++i) {

         auto pos = hits.at(i)->GetPosition();
         auto posOnCircle = pos - center;
         auto angleHit = posOnCircle.Phi();

         // Check for a move over -Y axis
         int currYSign = GetSign(posOnCircle.Y());

         // If we have moved over the Y axis in some direction
         // last = (0 or -1) and current = 1 -> numCross--
         // last = (0 or 1) and current = -1 -> numCross++
         if (posOnCircle.X() < 0 && lastYSign != currYSign) {
            numYCross -= currYSign;
         }
         lastYSign = currYSign;
         angleHit += 2 * M_PI * numYCross;

         whit.push_back(angleHit);
         arclength.push_back((radius * (refAng - whit.at(i))));

         arclengthGraph->SetPoint(arclengthGraph->GetN(), arclength.at(i), pos.Z());

         if (track.GetTrackID() > -1)
            LOG(debug2) << posOnCircle.X() << "  " << posOnCircle.Y() << " " << pos.Z() << " "
                        << hits.at(i)->GetTimeStamp() << " " << arclength.back() << "\n";

         // Add a hit in the Arc legnth - Z plane
         Double_t xPos = arclength.at(i);
         Double_t yPos = pos.Z();
         Double_t zPos = i * 1E-19;

         thetaHits.emplace_back(i, hits.at(i)->GetPadNum(), XYZPoint(xPos, yPos, zPos), hits.at(i)->GetCharge());
      }

      // TF1 *f1 = new TF1("f1", "pol1", -500, 500);
      // TF1 * f1 = new TF1("f1",[](double *x, double *p) { return (p[0]+p[1]*x[0]); },-500,500,2);
      // TF1 * f1 = new TF1("f1","[0]+[1]*x",-500,500);
      // TF1 *f1 = new TF1("f1", fitf, -500, 500, 2);
      // arclengthGraph->Fit(f1, "R");
      // auto slope = ROOT::Math::XYVector(1, f1->GetParameter(1)).Unit();
      // std::cout << " Slope " << slope << "\n";

      Double_t angle = 0.0;
      Double_t phi0 = 0.0;

      try {

         if (thetaHits.size() > 0) {
            // std::cout<<" RANSAC Theta "<<"\n";
            std::vector<AtTrack> thetaTracks;

            SampleConsensus::AtSampleConsensus RansacTheta;
            RansacTheta.SetPatternType(AtPatterns::PatternType::kLine);
            RansacTheta.SetMinHitsPattern(0.1 * thetaHits.size());
            RansacTheta.SetDistanceThreshold(6.0);
            RansacTheta.SetFitPattern(true);
            thetaTracks = RansacTheta.Solve(thetaHits).GetTrackCand();

            if (thetaTracks.size() > 0) {

               // NB: Only the most intense line is taken, if any
               auto line = dynamic_cast<const AtPatterns::AtPatternLine *>(thetaTracks.at(0).GetPattern());
               LOG(info) << "Track ID: " << track.GetTrackID() << " with " << track.GetHitArray().size()
                         << " hits. Fit " << thetaTracks[0].GetHitArray().size() << "/" << thetaHits.size()
                         << " hits in first of " << thetaTracks.size() << " tracks";

               auto dirTheta = line->GetDirection();
               auto dir2D = ROOT::Math::XYVector(dirTheta.X(), dirTheta.Y()).Unit();

               int sign = 0;

               if (dir2D.X() * dir2D.Y() < 0)
                  sign = -1;
               else
                  sign = 1;

               if (dir2D.X() != 0) {
                  angle = acos(sign * fabs(dir2D.Y())) * TMath::RadToDeg();
               }

               LOG(info) << "Setting theta geo to: " << angle << " with ransac direction: " << dir2D;
               for (int i = 1; i < thetaTracks.size(); ++i) {
                  auto ranLine = dynamic_cast<const AtPatterns::AtPatternLine *>(thetaTracks.at(i).GetPattern());
                  auto ranDir = ROOT::Math::XYVector(ranLine->GetDirection().X(), ranLine->GetDirection().Y()).Unit();
                  LOG(info) << "RANSAC direction " << i << " with " << thetaTracks[i].GetHitArray().size() << "/"
                            << thetaHits.size() << ": " << ranDir;
               }

               auto temp2 = posPCA - center;
               phi0 = TMath::ATan2(temp2.Y(), temp2.X());

            } // thetaTracks
            track.SetGeoTheta(angle * TMath::Pi() / 180.0);
            track.SetGeoPhi(phi0);

         } // if
         else {
            LOG(info) << "Track ID: " << track.GetTrackID() << " with " << track.GetHitArray().size()
                      << " hits does not have enough theta hits to get angle";
         }

      } catch (std::exception &e) {

         std::cout << " AtPRA::SetTrackInitialParameters - Exception caught : " << e.what() << "\n";
      }

   } // end if (!circularTracks->empty())
}

Double_t fitf(Double_t *x, Double_t *par)
{

   return par[0] + par[1] * x[0];
}

void AtPATTERN::AtPRA::PruneTrack(AtTrack &track)
{
   auto &hitArray = track.GetHitArray();

   std::cout << "    === Prunning track : " << track.GetTrackID() << "\n";
   std::cout << "      = Hit Array size : " << hitArray.size() << "\n";

   for (auto iHit = 0; iHit < hitArray.size(); ++iHit) {

      try {
         bool isNoise = kNN(hitArray, *hitArray.at(iHit), fKNN); // Returns true if hit is an outlier

         if (isNoise) {
            // std::cout<<" Hit "<<iHit<<" flagged as outlier. "<<"\n";
            hitArray.erase(hitArray.begin() + iHit);
         }
      } catch (std::exception &e) {

         std::cout << " AtPRA::PruneTrack - Exception caught : " << e.what() << "\n";
      }
   }

   std::cout << "      = Hit Array size after prunning : " << hitArray.size() << "\n";
}

bool AtPATTERN::AtPRA::kNN(const std::vector<std::unique_ptr<AtHit>> &hits, AtHit &hitRef, int k)
{

   std::vector<Double_t> distances;
   distances.reserve(hits.size());

   std::for_each(hits.begin(), hits.end(), [&distances, &hitRef](const std::unique_ptr<AtHit> &hit) {
      distances.push_back(TMath::Sqrt((hitRef.GetPosition() - hit->GetPosition()).Mag2()));
   });

   std::sort(distances.begin(), distances.end(), [](Double_t a, Double_t b) { return a < b; });

   Double_t mean = 0.0;
   Double_t stdDev = 0.0;

   if (k > hits.size())
      k = hits.size();

   // Compute mean distance of kNN
   for (auto i = 0; i < k; ++i)
      mean += distances.at(i);

   mean /= k;

   // Compute std dev
   for (auto i = 0; i < k; ++i)
      stdDev += TMath::Power((distances.at(i) - mean), 2);

   stdDev = TMath::Sqrt(stdDev / k);

   // Compute threshold
   Double_t T = mean + stdDev * fStdDevMulkNN;

   return (T < fkNNDist) ? false : true;
}

/*
void AtPATTERN::AtPRA::SetTrackCurvature(AtTrack &track)
{
   std::cout << " new track  "
             << "\n";
   std::vector<double> radius_vec;
   std::vector<AtHit> hitArray = track.GetHitArray();
   int nstep = 0.60 * hitArray.size(); // 20% of the hits to calculate the radius of curvature with less fluctuations

   for (Int_t iHit = 0; iHit < (hitArray.size() - nstep); iHit++) {

      AtHit hitA = hitArray.at(iHit);
      AtHit hitB = hitArray.at((int)(iHit + (nstep / 2.0)));
      AtHit hitC = hitArray.at((int)(iHit + nstep));

      // std::cout<<nstep<<" "<<iHit<<"  "<<(int)(iHit+(nstep/2.0))<<"  "<<(int)(iHit+nstep)<<"\n";

      auto posA = hitA.GetPosition();
      auto posB = hitB.GetPosition();
      auto posC = hitC.GetPosition();

      double slopeAB = (posB.Y() - posA.Y()) / (posB.X() - posA.X()); // m1
      double slopeBC = (posC.Y() - posB.Y()) / (posC.X() - posB.X()); // m2

      double centerX = (slopeAB * slopeBC * (posA.Y() - posC.Y()) + slopeBC * (posB.X() + posA.X()) -
                        slopeAB * (posB.X() + posC.X())) /
                       (2.0 * (slopeBC - slopeAB));

      double centerY = (-1 / slopeAB) * (centerX - (posB.X() + posA.X()) / 2.0) + (posB.Y() + posA.Y()) / 2.0;

      // std::cout<<" Center "<<centerX<<" - "<<centerY<<"\n";

      double radiusA = TMath::Sqrt(TMath::Power(posA.X() - centerX, 2) + TMath::Power(posA.Y() - centerY, 2));
      radius_vec.push_back(radiusA);
      double radiusB = TMath::Sqrt(TMath::Power(posB.X() - centerX, 2) + TMath::Power(posB.Y() - centerY, 2));
      radius_vec.push_back(radiusB);
      double radiusC = TMath::Sqrt(TMath::Power(posC.X() - centerX, 2) + TMath::Power(posC.Y() - centerY, 2));
      radius_vec.push_back(radiusC);
   }
}
*/
