#ifndef ATFINDVERTEX_H
#define ATFINDVERTEX_H

#include "AtTrack.h"
#include "AtPattern.h"    // for AtPattern
#include "AtPatternEvent.h"
#include "AtPatternLine.h"
#include "AtPatternTypes.h" // for PatternType, PatternTy...

#include <Rtypes.h>

#include <TObject.h>
#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacemenXYZVectorD
#include <Math/Vector3Dfwd.h> // for XYZVector

class TBuffer;
class TClass;
class TMemberInspector;
class AtTrack;

using XYZVector = ROOT::Math::XYZVector;

struct tracksFromVertex {
  XYZVector vertex;
  std::vector<AtTrack> tracks;
};

class AtFindVertex : public TObject {

public:
   AtFindVertex(Double_t lineDistThreshold);
   virtual ~AtFindVertex();


   void FindVertex(std::vector<AtTrack> tracks, Int_t nbTracksPerVtx);
   void FindVertexSingleLine(std::vector<AtTrack> tracks);
   void FindVertexMultipleLines(std::vector<AtTrack> tracks, Int_t nbTracksPerVtx);
   XYZVector ClosestPoint2Lines(XYZVector d1, XYZVector pt1, XYZVector d2, XYZVector pt2);
   std::vector<XYZVector> ClosestPointProjOnLines(XYZVector d1, XYZVector pt1, XYZVector d2, XYZVector pt2);
   std::vector<std::vector<Int_t>> SortTrackSameVtx(std::vector<std::vector<Double_t>> lines);
   std::vector<XYZVector> CoGVtx(std::vector<std::vector<Int_t>> vtxCand, std::vector<std::vector<Double_t>> lines, std::vector<Double_t> wlines);
   std::vector<std::pair<Int_t, XYZVector>> CoGVtxSingleTrack(std::vector<std::vector<Double_t>> lines, std::vector<Double_t> wlines);
   XYZVector ptOnLine(std::vector<Double_t> line, XYZVector pointToProj);
   Double_t distPtLine(XYZVector dir, XYZVector ptLine, XYZVector pt);


   void SetTracksVertex(tracksFromVertex val)
   {
      fTracksFromVertex.push_back(val);
   }

   void SetLineDistThreshold(Double_t val)
   {
      fLineDistThreshold = val;
   }

   void SetBeam(XYZVector pos, XYZVector dir)
   {
      fBeamPoint = pos;
      fBeamDir = dir;
   }

   std::vector<tracksFromVertex> GetTracksVertex(){ return fTracksFromVertex;}



private:

   std::vector<tracksFromVertex> fTracksFromVertex;

   Double_t fLineDistThreshold;
   XYZVector fBeamPoint;
   XYZVector fBeamDir;
   std::vector<std::vector<Double_t>> fLineCand;



   ClassDef(AtFindVertex, 1);
};

#endif //#ifndef AtFindVertex_H
