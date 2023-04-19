#ifndef ATFINDVERTEX_H
#define ATFINDVERTEX_H
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtTrack.h"

#include <Math/Vector3D.h>    // for DisplacemenXYZVectorD
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>

#include <algorithm> // for max
#include <utility>   // for pair
#include <vector>    // for vector

using XYZVector = ROOT::Math::XYZVector;

struct tracksFromVertex {
   XYZVector vertex;
   std::vector<AtTrack> tracks;
};

class AtFindVertex {

public:
   AtFindVertex(Double_t lineDistThreshold = 15);
   virtual ~AtFindVertex();

   void FindVertex(std::vector<AtTrack> tracks, Int_t nbTracksPerVtx);
   void FindVertexSingleLine(std::vector<AtTrack> tracks);
   void FindVertexMultipleLines(std::vector<AtTrack> tracks, Int_t nbTracksPerVtx);
   XYZVector ClosestPoint2Lines(std::vector<Double_t> line1, std::vector<Double_t> line2);
   std::vector<XYZVector> ClosestPointProjOnLines(std::vector<Double_t> line1, std::vector<Double_t> line2);
   std::vector<std::vector<Int_t>> SortTrackSameVtx(std::vector<std::vector<Double_t>> lines);
   std::vector<XYZVector> CoGVtx(std::vector<std::vector<Int_t>> vtxCand, std::vector<std::vector<Double_t>> lines,
                                 std::vector<Double_t> wlines);
   std::vector<std::pair<Int_t, XYZVector>>
   CoGVtxSingleTrack(std::vector<std::vector<Double_t>> lines, std::vector<Int_t> itracks);
   XYZVector ptOnLine(std::vector<Double_t> line, XYZVector pointToProj);
   Double_t distPtLine(XYZVector dir, XYZVector ptLine, XYZVector pt);
   Double_t distLines(std::vector<Double_t> line1, std::vector<Double_t> line2);
   Double_t angLines(std::vector<Double_t> line1, std::vector<Double_t> line2);
   Bool_t checkExclusivity(std::vector<Int_t> v1, std::vector<Int_t> v2);

   void SetTracksVertex(tracksFromVertex val) { fTracksFromVertex.push_back(val); }

   void SetLineDistThreshold(Double_t val) { fLineDistThreshold = val; }

   void SetBeam(XYZVector pos, XYZVector dir)
   {
      fBeamPoint = pos;
      fBeamDir = dir;
      fBeamLine.resize(6);
      fBeamLine[0] = fBeamPoint.X();
      fBeamLine[1] = fBeamPoint.Y();
      fBeamLine[2] = fBeamPoint.Z();
      fBeamLine[3] = fBeamDir.X();
      fBeamLine[4] = fBeamDir.Y();
      fBeamLine[5] = fBeamDir.Z();
   }

   std::vector<tracksFromVertex> GetTracksVertex() { return fTracksFromVertex; }

private:
   std::vector<tracksFromVertex> fTracksFromVertex;

   Double_t fLineDistThreshold;
   XYZVector fBeamPoint;
   XYZVector fBeamDir;
   std::vector<Double_t> fBeamLine;
   std::vector<std::vector<Double_t>> fLineCand;
};

#endif //#ifndef AtFindVertex_H
