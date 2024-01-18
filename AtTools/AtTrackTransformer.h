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

private:
};

} // namespace AtTools

#endif
