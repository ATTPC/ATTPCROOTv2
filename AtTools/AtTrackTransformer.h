#ifndef ATTRACKTRANSFORMER_H
#define ATTRACKTRANSFORMER_H

#include <Rtypes.h>

class AtTrack;

namespace AtTools {

class AtTrackTransformer {

public:
   AtTrackTransformer();
   ~AtTrackTransformer();

   void ClusterizeSmooth3D(AtTrack &track, Float_t radius, Float_t distance);
   const std::tuple<Double_t, Double_t> GetPIDFromHits(AtTrack &track, Double_t theta);

   Bool_t FindVertexTrack(AtTrack *trA, AtTrack *trB);

   Bool_t MergeTracks(std::vector<AtTrack *> *trackCandSource, std::vector<AtTrack> *trackDest,
                      Bool_t enableSingleVertexTrack, Double_t clusterRadius, Double_t clusterDistance);

private:
};

} // namespace AtTools

#endif
