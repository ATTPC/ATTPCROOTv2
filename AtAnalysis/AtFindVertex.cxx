/*
Move this to Fitter folder, the vertex determination will go with the fitter
*/
#include "AtFindVertex.h"

#include "AtTrack.h"
#include "AtPattern.h"    // for AtPattern
#include "AtPatternEvent.h"
#include "AtPatternLine.h"
#include "AtPatternTypes.h" // for PatternType, PatternTy...


#include <algorithm>
#include <cmath>    // for sqrt
#include <iostream> // for operator<<, basic_ostream::operator<<
#include <iterator> // for back_insert_iterator, back_inserter
#include <memory>   // for shared_ptr, __shared_ptr_access, __sha...
#include <utility>  // for pair

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacemenXYZVectorD
#include <Math/Vector3Dfwd.h> // for XYZVector


#include <FairLogger.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";


ClassImp(AtFindVertex);


AtFindVertex::AtFindVertex(Double_t lineDistThreshold=15)
   : fLineDistThreshold(lineDistThreshold), fTracksFromVertex(0)
{
  fBeamPoint.SetXYZ(0,0,500);
  fBeamDir.SetXYZ(0,0,1);
}


AtFindVertex::~AtFindVertex() = default;


void AtFindVertex::FindVertex(std::vector<AtTrack> tracks, Int_t nbTracksPerVtx)
{
  if(tracks.size() < nbTracksPerVtx) return;
  switch (nbTracksPerVtx) {
    case 1:
      FindVertexSingleLine(tracks);
    break;
    default:
      FindVertexMultipleLines(tracks, nbTracksPerVtx);
  }

}


void AtFindVertex::FindVertexSingleLine(std::vector<AtTrack> tracks)
{
  std::vector<std::vector<Double_t>> lines;
  std::vector<Double_t> wlines;//weights for CoG (ex: Chi2, chargeTot, nbInliners...)
  std::vector<Int_t> itracks;
  Int_t it=-1;
  for (auto track : tracks) {
    it++;
    auto ransacLine = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
    std::vector<Double_t> patternPar;
    patternPar = ransacLine->GetPatternPar();
    if(patternPar.size()==0) continue;
    Bool_t nanPattern=kFALSE;
    for (Int_t i = 0; i < patternPar.size(); i++)
      if (patternPar.at(i)!=patternPar.at(i)) nanPattern=kTRUE;
    if (nanPattern) continue;
    lines.push_back(patternPar);
    wlines.push_back(ransacLine->GetNumPoints());
    itracks.push_back(it);
  }
    std::vector<std::pair<Int_t, XYZVector>> cogVtx;
    cogVtx = CoGVtxSingleTrack(lines, wlines,itracks);
    for(Int_t i =0; i<cogVtx.size(); i++)std::cout<<cogVtx.size()<<" check cogVtx size "<<cogVtx.at(i).second.X()<<" "<<cogVtx.at(i).second.Y()<<" "<<cogVtx.at(i).second.Z()<<std::endl;

    for (Int_t i = 0; i < cogVtx.size(); i++) {
      if(cogVtx.at(i).second.X()!=cogVtx.at(i).second.X() ||
      cogVtx.at(i).second.Y()!=cogVtx.at(i).second.Y() ||
      cogVtx.at(i).second.Z()!=cogVtx.at(i).second.Z()) continue;
      if(cogVtx.at(i).second.Z()<0 || cogVtx.at(i).second.Z()>1000 || sqrt(cogVtx.at(i).second.Perp2())>30) continue;
      std::vector<AtTrack> tracksVtx;
      tracksVtx.push_back(tracks.at(cogVtx.at(i).first));
      tracksFromVertex tv;
      tv.vertex = cogVtx.at(i).second;
      tv.tracks = tracksVtx;
      SetTracksVertex(tv);
    }
}

void AtFindVertex::FindVertexMultipleLines(std::vector<AtTrack> tracks, Int_t nbTracksPerVtx)
{
  std::vector<std::vector<Double_t>> lines;
  std::vector<Double_t> wlines;//weights for CoG (ex: Chi2, chargeTot, nbInliners...)
  for (auto track : tracks) {
    auto ransacLine = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
    std::vector<Double_t> patternPar;
    patternPar = ransacLine->GetPatternPar();
    if(patternPar.size()==0) continue;
    Bool_t nanPattern=kFALSE;
    for (Int_t i = 0; i < patternPar.size(); i++)
      if (patternPar.at(i)!=patternPar.at(i)) nanPattern=kTRUE;
    if (nanPattern) continue;
    lines.push_back(patternPar);
    wlines.push_back(ransacLine->GetChi2());
  }

    std::vector<AtTrack> tracksFromSameVtx;//save tracks coming from a common vertex
    std::vector<std::vector<Int_t>> vtxCand;//save track ID of tracks coming from a common vertex
    vtxCand = SortTrackSameVtx(lines);
    // for(Int_t i =0; i<vtxCand.size(); i++)std::cout<<vtxCand.size()<<" check vtxCand size "<<vtxCand.at(i).size()<<std::endl;

    std::vector<XYZVector> cogVtx;
    cogVtx = CoGVtx(vtxCand, lines, wlines);
    // for(Int_t i =0; i<cogVtx.size(); i++)std::cout<<cogVtx.size()<<" check cogVtx size "<<cogVtx.at(i).X()<<" "<<cogVtx.at(i).Y()<<" "<<cogVtx.at(i).Z()<<std::endl;

    if(vtxCand.size()!=cogVtx.size()) LOG(WARNING)<<cYELLOW<<"AtFindVertex : vtxCand.size() != cogVtx.size()"<<cNORMAL<<std::endl;

    for (Int_t i = 0; i < vtxCand.size(); i++) {
      if(cogVtx.at(i).X()!=cogVtx.at(i).X() || cogVtx.at(i).Y()!=cogVtx.at(i).Y() || cogVtx.at(i).Z()!=cogVtx.at(i).Z()) continue;
      if(cogVtx.at(i).Z()<0 || cogVtx.at(i).Z()>1000 || sqrt(cogVtx.at(i).Perp2())>30) continue;
      if(vtxCand.at(i).size()>2) std::cout<<cYELLOW<<" vtx with more than 2 tracks("<<vtxCand.at(i).size()<<")"<<cNORMAL<<std::endl;
      std::vector<AtTrack> tracksVtx;
      for (size_t j = 0; j < vtxCand.at(i).size(); j++) {
        tracksVtx.push_back(tracks.at(vtxCand.at(i).at(j)));
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
  std::vector<std::vector<Bool_t>> paired;
  std::vector<Bool_t> buff;
  buff.resize(lines.size(),kFALSE);
  paired.resize(lines.size(),buff);
  // Test each line against the others
  for(Int_t i=0;i<lines.size()-1;i++)
  {
      std::vector<Double_t> line = lines.at(i);
      XYZVector p1(line.at(0), line.at(1), line.at(2));
      XYZVector d1(line.at(3), line.at(4), line.at(5));

      std::vector<Int_t> trackIDOneVtx;
      trackIDOneVtx.push_back(i);
      XYZVector buffVtx(0.,0.,9999.);

      //std::cout<<cYELLOW<<p1.X()<<" "<<d1.X()<<" after p1, d1 "<<cNORMAL<<std::endl;

                for(Int_t j=i+1; j<UInt_t(lines.size());j++)
                {
                  std::vector<Double_t> line_f = lines.at(j);
                  XYZVector p2(line_f.at(0), line_f.at(1), line_f.at(2));
                  XYZVector d2(line_f.at(3), line_f.at(4), line_f.at(5));

                  if(buffVtx.Z()>999) {
                    // Double_t angle = d1.Angle(d2)*180./3.1415;
                    Double_t angle = acos(d1.Dot(d2)/(sqrt(d1.Mag2())*sqrt(d2.Mag2())))*180./3.1415;
                    XYZVector n = d1.Cross(d2);
                    Double_t sdist = fabs( n.Dot(p1-p2)/sqrt(n.Mag2()) );

                    if(sdist < fLineDistThreshold){
                      if(angle<10 || angle>170){
                        XYZVector vtxLines1 = ClosestPoint2Lines(d1, p1, fBeamDir, fBeamPoint);
                        XYZVector vtxLines2 = ClosestPoint2Lines(d2, p2, fBeamDir, fBeamPoint);
                        buffVtx = 0.5*(vtxLines1+vtxLines2);
                      }
                      else{
                        buffVtx = ClosestPoint2Lines(d1, p1, d2, p2);
                      }
                      if(!paired[i][j] || !paired[j][i]) {
                        trackIDOneVtx.push_back(j);
                      }
                    }
                  }
                  else if (buffVtx.Z()<999) {
                    XYZVector projBuffVtx;
                    projBuffVtx = ptOnLine(line_f, buffVtx);
                    Double_t sdist = sqrt((buffVtx-projBuffVtx).Mag2());//distance between buffVtx and line_f
                    if(sdist < fLineDistThreshold/2){
                      if(!paired[i][j] || !paired[j][i]) {
                        trackIDOneVtx.push_back(j);
                      }
                    }
                  }
                }//j loop lines
                result.push_back(trackIDOneVtx);
                for(int ii=0; ii<trackIDOneVtx.size()-1;ii++){
                  for(int jj=ii+1; jj<trackIDOneVtx.size();jj++){
                    paired[trackIDOneVtx.at(ii)][trackIDOneVtx.at(jj)] = kTRUE;
                    paired[trackIDOneVtx.at(jj)][trackIDOneVtx.at(ii)] = kTRUE;
                    // std::cout<<" ii "<<ii<<" jj "<<jj<<" "<<paired[trackIDOneVtx.at(ii)][trackIDOneVtx.at(jj)]<<" "<<paired[trackIDOneVtx.at(jj)][trackIDOneVtx.at(ii)]<<std::endl;
                  }
                }

  }//i loop lines
  //std::cout<<" end loop i "<<result.size()<<std::endl;

  return result;
}


std::vector<std::pair<Int_t, XYZVector>> AtFindVertex::CoGVtxSingleTrack(std::vector<std::vector<Double_t>> lines, std::vector<Double_t> wlines, std::vector<Int_t> itracks)
{
  std::vector<std::pair<Int_t, XYZVector>> result;



  for(Int_t i=0;i<lines.size();i++)
  {
    XYZVector CoG(0,0,0);
    std::cout<<lines.size()<<" check lines size "<<std::endl;
      std::vector<Double_t> line = lines.at(i);
      XYZVector p1(line.at(0), line.at(1), line.at(2));
      XYZVector d1(line.at(3), line.at(4), line.at(5));
      // Double_t angle = d1.Angle(fBeamDir)*180./3.1415;
      Double_t angle = acos(d1.Dot(fBeamDir)/(sqrt(d1.Mag2())*sqrt(fBeamDir.Mag2())))*180./3.1415;
      XYZVector n = d1.Cross(fBeamDir);
      Double_t sdist = fabs( n.Dot(p1-fBeamPoint)/sqrt(n.Mag2()) );
      if(sdist < fLineDistThreshold){
          std::vector<XYZVector> projVtxOnLines1 = ClosestPointProjOnLines(d1, p1, fBeamDir, fBeamPoint);
          //following lines are also a possible solution
          // CoG += projVtxOnLines1.at(0)*(pow(wlines.at(i),2));
          // CoG += projVtxOnLines1.at(1)*(pow(10,2));//if a track has nbHits>10 the vertex will be closer to the line than to the beam
          // CoG = CoG*(1./(pow(wlines.at(i),2)+pow(10,2)));
          CoG = projVtxOnLines1.at(0);//this way the track angle is better than taking the CoG, only the range has uncertainty
          auto p = std::make_pair(itracks.at(i), CoG);
          result.push_back(p);
      }
   }// Loop over the lines (for loop i)

   return result;
}


std::vector<XYZVector> AtFindVertex::CoGVtx(std::vector<std::vector<Int_t>> vtxCand, std::vector<std::vector<Double_t>> lines, std::vector<Double_t> wlines)
{
  std::vector<XYZVector> result;

  //loop on vtx Candidates
  for (size_t iv = 0; iv < vtxCand.size(); iv++) {

  XYZVector CoG(0,0,0);

  std::vector<XYZVector> sumVtx;//sum of points corresponding to where are the closest distances between each lines
  XYZVector buffVec1(0,0,0);
  sumVtx.resize(vtxCand.at(iv).size(),buffVec1);

  for(Int_t i=0;i<vtxCand.at(iv).size()-1;i++)
  {
      Int_t ii = vtxCand.at(iv).at(i);
      std::vector<Double_t> line = lines.at(ii);
      XYZVector p1(line.at(0), line.at(1), line.at(2));
      XYZVector d1(line.at(3), line.at(4), line.at(5));

                for(Int_t j=i+1; j<vtxCand.at(iv).size();j++)
                {
                    Int_t jj = vtxCand.at(iv).at(j);
                    std::vector<Double_t> line_f = lines.at(jj);
                          XYZVector p2(line_f.at(0), line_f.at(1), line_f.at(2));
                          XYZVector d2(line_f.at(3), line_f.at(4), line_f.at(5));
                          // Double_t angle = d1.Angle(d2)*180./3.1415;
                          Double_t angle = acos(d1.Dot(d2)/(sqrt(d1.Mag2())*sqrt(d2.Mag2())))*180./3.1415;

                          XYZVector n = d1.Cross(d2);
                          Double_t sdist = fabs( n.Dot(p1-p2)/sqrt(n.Mag2()) );
                          if(sdist < fLineDistThreshold){
                            if(angle<10 || angle>170){
                              std::vector<XYZVector> projVtxOnLines1 = ClosestPointProjOnLines(d1, p1, fBeamDir, fBeamPoint);
                              std::vector<XYZVector> projVtxOnLines2 = ClosestPointProjOnLines(d2, p2, fBeamDir, fBeamPoint);
                              sumVtx.at(i) += projVtxOnLines1.at(0);
                              sumVtx.at(j) += projVtxOnLines2.at(0);
                            }
                            else{
                              std::vector<XYZVector> projVtxOnLines = ClosestPointProjOnLines(d1, p1, d2, p2);
                              sumVtx.at(i) += projVtxOnLines.at(0);
                              sumVtx.at(j) += projVtxOnLines.at(1);
                              // std::cout<<"projVtxOnLines (i) "<<projVtxOnLines.at(0).X()<<" "<<projVtxOnLines.at(0).Y()<<" "<<projVtxOnLines.at(0).Z()<<" "<<std::endl;
                              // std::cout<<"projVtxOnLines (j) "<<projVtxOnLines.at(1).X()<<" "<<projVtxOnLines.at(1).Y()<<" "<<projVtxOnLines.at(1).Z()<<" "<<std::endl;
                            }
                          }
                 }// End of track_f (for loop j)
   }// Loop over the lines (for loop i)

   Double_t sumW=0; //sum of the weights
   for(Int_t i=0;i<vtxCand.at(iv).size();i++){
     // std::cout<<"sumvtx "<<sumVtx.at(i).X()<<" "<<sumVtx.at(i).Y()<<" "<<sumVtx.at(i).Z()<<" chi2 "<<wlines.at(vtxCand.at(iv).at(i))<<std::endl;
      CoG += sumVtx.at(i)*(1./(vtxCand.at(iv).size()-1))*(1./wlines.at(vtxCand.at(iv).at(i)));
      sumW += (Double_t)(1./wlines.at(vtxCand.at(iv).at(i)));
   }
   CoG = CoG*(1./sumW);
   std::cout<<" vertex "<<CoG.X()<<" "<<CoG.Y()<<" "<<CoG.Z()<<std::endl;
   result.push_back(CoG);

   }//for iv vertex candidates

   return result;
}

//returns the mean point at the closest distance between two lines
XYZVector AtFindVertex::ClosestPoint2Lines(XYZVector d1, XYZVector pt1, XYZVector d2, XYZVector pt2)
{
  XYZVector n1 = d1.Cross(d2.Cross(d1));
  XYZVector n2 = d2.Cross(d1.Cross(d2));
  Double_t t1 = (pt2-pt1).Dot(n2)/(d1.Dot(n2));
  Double_t t2 = (pt1-pt2).Dot(n1)/(d2.Dot(n1));
  XYZVector c1 = pt1 + t1*d1;
  XYZVector c2 = pt2 + t2*d2;
  XYZVector meanpoint = 0.5*(c1+c2);

  return meanpoint;

}

//returns the projections on each lines of the mean point at the closest distance between two lines
std::vector<XYZVector> AtFindVertex::ClosestPointProjOnLines(XYZVector d1, XYZVector pt1, XYZVector d2, XYZVector pt2)
{
  std::vector<XYZVector> result;
  XYZVector n1 = d1.Cross(d2.Cross(d1));
  XYZVector n2 = d2.Cross(d1.Cross(d2));
  Double_t t1 = (pt2-pt1).Dot(n2)/(d1.Dot(n2));
  Double_t t2 = (pt1-pt2).Dot(n1)/(d2.Dot(n1));
  XYZVector c1 = pt1 + t1*d1;
  XYZVector c2 = pt2 + t2*d2;
  result.push_back(c1);
  result.push_back(c2);

  return result;

}

//returns the projection of a point on the parametric line
XYZVector AtFindVertex::ptOnLine(std::vector<Double_t> line, XYZVector pointToProj)
{
        XYZVector result(-999,-999,-999);
        XYZVector posOn(line.at(0), line.at(1), line.at(2));
        XYZVector dir(line.at(3), line.at(4), line.at(5));
        XYZVector vop1 = ((dir.Cross(pointToProj-posOn)).Cross(dir)).Unit();
        Double_t paraVar1 = pointToProj.Dot(dir.Unit())-posOn.Dot(dir.Unit());
        Double_t paraVar2 = posOn.Dot(vop1)-pointToProj.Dot(vop1);
        XYZVector vInter1 = posOn + dir.Unit()*paraVar1;
        XYZVector vInter2 = pointToProj + vop1*paraVar2;
        if(sqrt((vInter1-vInter2).Mag2())<1e-6) result=vInter1;

        return result;
}

//returns the distance between a point and a parametric line
Double_t AtFindVertex::distPtLine(XYZVector dir, XYZVector ptLine, XYZVector pt)
{
        return sqrt((ptLine-pt).Cross(dir).Mag2())/sqrt(dir.Mag2());
}
