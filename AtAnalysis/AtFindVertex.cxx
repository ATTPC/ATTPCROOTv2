/*
Move this to Fitter folder, the vertex determination will go with the fitter
*/
#include "AtFindVertex.h"
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtPattern.h" // for AtPattern
#include "AtPatternLine.h"
#include "AtTrack.h"

#include <FairLogger.h>

#include <Math/Vector3D.h> // for DisplacemenXYZVectorD

#include <algorithm>
#include <cmath>    // for sqrt
#include <iostream> // for operator<<, basic_ostream::operator<<
#include <iterator> // for back_insert_iterator, back_inserter
#include <memory>   // for shared_ptr, __shared_ptr_access, __sha...
#include <utility>  // for pair

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

ClassImp(AtFindVertex);

AtFindVertex::AtFindVertex(Double_t lineDistThreshold) : fLineDistThreshold(lineDistThreshold), fTracksFromVertex(0)
{
   fBeamPoint.SetXYZ(0, 0, 500);
   fBeamDir.SetXYZ(0, 0, 1);
   fBeamLine.resize(6);
   fBeamLine[0] = fBeamPoint.X();
   fBeamLine[1] = fBeamPoint.Y();
   fBeamLine[2] = fBeamPoint.Z();
   fBeamLine[3] = fBeamDir.X();
   fBeamLine[4] = fBeamDir.Y();
   fBeamLine[5] = fBeamDir.Z();
}

AtFindVertex::~AtFindVertex() = default;

void AtFindVertex::FindVertex(std::vector<AtTrack> tracks, Int_t nbTracksPerVtx)
{
   if (tracks.size() < nbTracksPerVtx)
      return;
   switch (nbTracksPerVtx) {
   case 1: FindVertexSingleLine(tracks); break;
   default: FindVertexMultipleLines(tracks, nbTracksPerVtx);
   }
}

void AtFindVertex::FindVertexSingleLine(std::vector<AtTrack> tracks)
{
   std::vector<std::vector<Double_t>> lines;
   std::vector<Int_t> itracks;
   Int_t it = -1;
   for (auto track : tracks) {
      it++;
      auto ransacLine = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
      std::vector<Double_t> patternPar;
      patternPar = ransacLine->GetPatternPar();
      if (patternPar.size() == 0)
         continue;
      lines.push_back(patternPar);
      itracks.push_back(it);
   }
   std::vector<std::pair<Int_t, XYZVector>> cogVtx;
   cogVtx = CoGVtxSingleTrack(lines, itracks);
   // for(Int_t i =0; i<cogVtx.size(); i++)std::cout<<cogVtx.size()<<" check cogVtx size "<<cogVtx.at(i).second.X()<<"
   // "<<cogVtx.at(i).second.Y()<<" "<<cogVtx.at(i).second.Z()<<std::endl;

   for (auto &[num, pos] : cogVtx) {
      if (pos.Z() <= 0 || pos.Z() >= 1000 || sqrt(pos.Perp2()) > 30)
         continue;
      std::vector<AtTrack> tracksVtx;
      tracksVtx.push_back(tracks.at(num));
      tracksFromVertex tv;
      tv.vertex = pos;
      tv.tracks = tracksVtx;
      SetTracksVertex(tv);
   }
}

void AtFindVertex::FindVertexMultipleLines(std::vector<AtTrack> tracks, Int_t nbTracksPerVtx)
{
   std::vector<std::vector<Double_t>> lines;
   std::vector<Double_t> wlines; // weights for CoG (ex: Chi2, chargeTot, nbInliners...)
   for (auto track : tracks) {
      auto ransacLine = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
      std::vector<Double_t> patternPar;
      patternPar = ransacLine->GetPatternPar();
      if (patternPar.size() == 0)
         continue;
      lines.push_back(patternPar);
      wlines.push_back(ransacLine->GetChi2());
      // wlines.push_back(ransacLine->GetNumPoints());
      // wlines.push_back(1.);
   }

   std::vector<AtTrack> tracksFromSameVtx;  // save tracks coming from a common vertex
   std::vector<std::vector<Int_t>> vtxCand; // save track ID of tracks coming from a common vertex
   vtxCand = SortTrackSameVtx(lines);
   for (Int_t i = 0; i < vtxCand.size(); i++)
      std::cout << vtxCand.size() << " check vtxCand size " << vtxCand.at(i).size() << std::endl;

   std::vector<XYZVector> cogVtx;
   cogVtx = CoGVtx(vtxCand, lines, wlines);
   // for(Int_t i =0; i<cogVtx.size(); i++)std::cout<<cogVtx.size()<<" check cogVtx size "<<cogVtx.at(i).X()<<"
   // "<<cogVtx.at(i).Y()<<" "<<cogVtx.at(i).Z()<<std::endl;

   if (vtxCand.size() != cogVtx.size())
      LOG(warning) << cYELLOW << "AtFindVertex : vtxCand.size() != cogVtx.size()" << cNORMAL << std::endl;

   for (Int_t i = 0; i < vtxCand.size(); i++) {
      if (cogVtx.at(i).X() != cogVtx.at(i).X() || cogVtx.at(i).Y() != cogVtx.at(i).Y() ||
          cogVtx.at(i).Z() != cogVtx.at(i).Z())
         continue;
      if (cogVtx.at(i).Z() <= 0 || cogVtx.at(i).Z() >= 1000 || sqrt(cogVtx.at(i).Perp2()) > 30)
         continue;
      if (vtxCand.at(i).size() > nbTracksPerVtx)
         std::cout << cYELLOW << " vtx with more than " << nbTracksPerVtx << " tracks(" << vtxCand.at(i).size() << ")"
                   << cNORMAL << std::endl;
      std::vector<AtTrack> tracksVtx;
      for (auto vtxInd : vtxCand.at(i)) {
         tracksVtx.push_back(tracks.at(vtxInd));
      }
      tracksFromVertex tv;
      tv.vertex = cogVtx.at(i);
      tv.tracks = tracksVtx;
      SetTracksVertex(tv);
   }
}

std::vector<std::vector<Int_t>> AtFindVertex::SortTrackSameVtx(std::vector<std::vector<Double_t>> lines)
{
   std::vector<std::vector<Int_t>> result;
   std::vector<Int_t> paired;
   std::vector<XYZVector> vtx;

   // Test lines against each others
   for (Int_t i = 0; i < lines.size() - 1; i++) {
      if (std::find(paired.begin(), paired.end(), i) != paired.end())
         continue;
      std::vector<Double_t> line = lines.at(i);

      for (Int_t j = i + 1; j < lines.size(); j++) {
         if (std::find(paired.begin(), paired.end(), j) != paired.end())
            continue;

         XYZVector buffVtx(0., 0., 9999.);
         std::vector<Double_t> line_f = lines.at(j);

         if (vtx.size() == 0) { // if no vertex found
            Double_t angle = angLines(line, line_f);
            Double_t dist = distLines(line, line_f);

            if (dist < fLineDistThreshold) { // lines.at(i) and lines.at(j) close enough to make a new vertex
               if (angle < 10 || angle > 170) {
                  XYZVector vtxLines1 = ClosestPoint2Lines(line, fBeamLine);
                  XYZVector vtxLines2 = ClosestPoint2Lines(line_f, fBeamLine);
                  buffVtx = 0.5 * (vtxLines1 + vtxLines2);
               } else {
                  buffVtx = ClosestPoint2Lines(line, line_f);
               }
               // std::cout<<" paired size "<<paired.size()<<" "<<i<<" "<<j<<" "<<std::endl;
               paired.push_back(i);
               paired.push_back(j);
               vtx.push_back(buffVtx);
            }
         } else { // if already a vertex
            // std::cout<<" paired size "<<paired.size()<<" "<<i<<" "<<j<<" "<<std::endl;
            XYZVector projVtx;
            projVtx = ptOnLine(
               line_f, vtx.back()); // vtx.back() assumes one only wants events with one vertex, but it can be modified
            Double_t dist1 =
               sqrt((vtx.back() - projVtx).Mag2()); // distance between saved vtx and its projection on the new line
            Double_t radProjVtx = sqrt((0.5 * (vtx.back() + projVtx)).Perp2());
            // std::cout<<radProjVtx<<" distlines "<<dist1<<" vtxBack "<<vtx.back().X()<<" "<<vtx.back().Y()<<"
            // "<<vtx.back().Z()<<" "<<projVtx.X()<<" "<<projVtx.Y()<<" "<<projVtx.Z()<<std::endl;
            if (dist1 < fLineDistThreshold &&
                radProjVtx <= 30.) { // more than 2 lines from a same vertex in the beam region
               if (std::find(paired.begin(), paired.end(), i) == paired.end()) {
                  paired.push_back(i);
               }
               paired.push_back(j);
            }
         }
      } // j loop lines
   }    // i loop lines
   if (paired.size() > 1)
      result.push_back(paired);

   // for (size_t ipaired = 0; ipaired < paired.size(); ipaired++) {
   //   std::cout<<paired.at(ipaired)<<std::endl;
   // }

   return result;
}

std::vector<std::pair<Int_t, XYZVector>>
AtFindVertex::CoGVtxSingleTrack(std::vector<std::vector<Double_t>> lines, std::vector<Int_t> itracks)
{
   std::vector<std::pair<Int_t, XYZVector>> result;
   for (Int_t i = 0; i < lines.size(); i++) {
      XYZVector vtx(0, 0, 0);
      // std::cout<<lines.size()<<" check lines size "<<std::endl;
      std::vector<Double_t> line = lines.at(i);
      // Double_t angle = angLines(line, fBeamLine);
      Double_t dist = distLines(line, fBeamLine);
      if (dist < fLineDistThreshold) {
         std::vector<XYZVector> projVtxOnLines1 = ClosestPointProjOnLines(line, fBeamLine);
         vtx = projVtxOnLines1.at(
            0); // this way the track angle is better than taking the CoG, the uncertainty on the range is not improved
         auto p = std::make_pair(itracks.at(i), vtx);
         result.push_back(p);
      }
   } // loop i

   return result;
}

std::vector<XYZVector> AtFindVertex::CoGVtx(std::vector<std::vector<Int_t>> vtxCand,
                                            std::vector<std::vector<Double_t>> lines, std::vector<Double_t> wlines)
{
   std::vector<XYZVector> result;

   // loop on vtx Candidates
   for (auto iv : vtxCand) {

      XYZVector CoG(0, 0, 0);

      std::vector<XYZVector> sumVtx; // sum of points corresponding to where are the closest distances between each
                                     // lines
      XYZVector buffVec1(0, 0, 0);
      sumVtx.resize(iv.size(), buffVec1);

      for (Int_t i = 0; i < iv.size() - 1; i++) {
         Int_t ii = iv.at(i);
         std::vector<Double_t> line = lines.at(ii);

         for (Int_t j = i + 1; j < iv.size(); j++) {
            Int_t jj = iv.at(j);
            std::vector<Double_t> line_f = lines.at(jj);

            Double_t angle = angLines(line, line_f);
            Double_t dist = distLines(line, line_f);
            if (dist < fLineDistThreshold) {
               if (angle < 10 || angle > 170) {
                  std::vector<XYZVector> projVtxOnLines1 = ClosestPointProjOnLines(line, fBeamLine);
                  std::vector<XYZVector> projVtxOnLines2 = ClosestPointProjOnLines(line_f, fBeamLine);
                  sumVtx.at(i) += projVtxOnLines1.at(0);
                  sumVtx.at(j) += projVtxOnLines2.at(0);
               } else {
                  std::vector<XYZVector> projVtxOnLines = ClosestPointProjOnLines(line, line_f);
                  sumVtx.at(i) += projVtxOnLines.at(0);
                  sumVtx.at(j) += projVtxOnLines.at(1);
                  // std::cout<<"check CoG "<<line.at(0)<<" "<<line.at(1)<<" "<<line_f.at(0)<<"
                  // "<<line_f.at(1)<<std::endl;
               }
            }
         } // End of track_f (for loop j)
      }    // Loop over the lines (for loop i)

      Double_t sumW = 0; // sum of the weights
      for (Int_t i = 0; i < iv.size(); i++) {
         // std::cout<<"sumvtx "<<sumVtx.at(i).X()<<" "<<sumVtx.at(i).Y()<<" "<<sumVtx.at(i).Z()<<" chi2
         // "<<wlines.at(iv.at(i))<<std::endl;
         CoG += sumVtx.at(i) * (1. / wlines.at(iv.at(i)));
         sumW += (Double_t)(1. / wlines.at(iv.at(i)));
      }
      CoG = CoG * (1. / (iv.size() - 1)) * (1. / sumW);
      std::cout << " vertex " << CoG.X() << " " << CoG.Y() << " " << CoG.Z() << std::endl;
      result.push_back(CoG);

   } // for iv vertex candidates

   return result;
}

// returns the mean point at the closest distance between two lines
XYZVector AtFindVertex::ClosestPoint2Lines(std::vector<Double_t> line1, std::vector<Double_t> line2)
{
   XYZVector p1(line1.at(0), line1.at(1), line1.at(2));
   XYZVector d1(line1.at(3), line1.at(4), line1.at(5));
   XYZVector p2(line2.at(0), line2.at(1), line2.at(2));
   XYZVector d2(line2.at(3), line2.at(4), line2.at(5));
   XYZVector n1 = d1.Cross(d2.Cross(d1));
   XYZVector n2 = d2.Cross(d1.Cross(d2));
   Double_t t1 = (p2 - p1).Dot(n2) / (d1.Dot(n2));
   Double_t t2 = (p1 - p2).Dot(n1) / (d2.Dot(n1));
   XYZVector c1 = p1 + t1 * d1;
   XYZVector c2 = p2 + t2 * d2;
   XYZVector meanpoint = 0.5 * (c1 + c2);

   return meanpoint;
}

// returns the projections on each lines of the mean point at the closest distance between two lines
std::vector<XYZVector> AtFindVertex::ClosestPointProjOnLines(std::vector<Double_t> line1, std::vector<Double_t> line2)
{
   std::vector<XYZVector> result;
   XYZVector p1(line1.at(0), line1.at(1), line1.at(2));
   XYZVector d1(line1.at(3), line1.at(4), line1.at(5));
   XYZVector p2(line2.at(0), line2.at(1), line2.at(2));
   XYZVector d2(line2.at(3), line2.at(4), line2.at(5));
   XYZVector n1 = d1.Cross(d2.Cross(d1));
   XYZVector n2 = d2.Cross(d1.Cross(d2));
   Double_t t1 = (p2 - p1).Dot(n2) / (d1.Dot(n2));
   Double_t t2 = (p1 - p2).Dot(n1) / (d2.Dot(n1));
   XYZVector c1 = p1 + t1 * d1;
   XYZVector c2 = p2 + t2 * d2;
   result.push_back(c1);
   result.push_back(c2);

   return result;
}

// returns the projection of a point on the parametric line
XYZVector AtFindVertex::ptOnLine(std::vector<Double_t> line, XYZVector pointToProj)
{
   XYZVector result(-999, -999, -999);
   XYZVector posOn(line.at(0), line.at(1), line.at(2));
   XYZVector dir(line.at(3), line.at(4), line.at(5));
   XYZVector vop1 = ((dir.Cross(pointToProj - posOn)).Cross(dir)).Unit();
   Double_t paraVar1 = pointToProj.Dot(dir.Unit()) - posOn.Dot(dir.Unit());
   Double_t paraVar2 = posOn.Dot(vop1) - pointToProj.Dot(vop1);
   XYZVector vInter1 = posOn + dir.Unit() * paraVar1;
   XYZVector vInter2 = pointToProj + vop1 * paraVar2;
   if (sqrt((vInter1 - vInter2).Mag2()) < 1e-6)
      result = vInter1;

   return result;
}

// returns the distance between a point and a parametric line
Double_t AtFindVertex::distPtLine(XYZVector dir, XYZVector ptLine, XYZVector pt)
{
   return sqrt((ptLine - pt).Cross(dir).Mag2()) / sqrt(dir.Mag2());
}

// returns the distance between two lines
Double_t AtFindVertex::distLines(std::vector<Double_t> line1, std::vector<Double_t> line2)
{
   XYZVector p1(line1.at(0), line1.at(1), line1.at(2));
   XYZVector d1(line1.at(3), line1.at(4), line1.at(5));
   XYZVector p2(line2.at(0), line2.at(1), line2.at(2));
   XYZVector d2(line2.at(3), line2.at(4), line2.at(5));
   XYZVector n = d1.Cross(d2);

   return fabs(n.Dot(p1 - p2) / sqrt(n.Mag2()));
}

// returns the angle between two lines
Double_t AtFindVertex::angLines(std::vector<Double_t> line1, std::vector<Double_t> line2)
{
   XYZVector d1(line1.at(3), line1.at(4), line1.at(5));
   XYZVector d2(line2.at(3), line2.at(4), line2.at(5));

   return acos(d1.Dot(d2) / (sqrt(d1.Mag2()) * sqrt(d2.Mag2()))) * 180. / 3.1415;
}

// check if a track already makes a vertex
Bool_t AtFindVertex::checkExclusivity(std::vector<Int_t> v1, std::vector<Int_t> v2)
{
   Bool_t result = kTRUE;
   std::vector<Int_t> diff;
   std::set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), std::inserter(diff, diff.begin()));
   if (v1.size() != diff.size())
      result = kFALSE;

   return result;
}
