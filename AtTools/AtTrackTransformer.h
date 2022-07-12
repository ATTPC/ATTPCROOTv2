#ifndef ATTRACKTRANSFORMER_H
#define ATTRACKTRANSFORMER_H

#include <Rtypes.h>

class AtTrack;

namespace AtTools {

class AtTrackTransformer {

public:
   AtTrackTransformer();
   ~AtTrackTransformer();

   void ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius);

private:
};

} // namespace AtTools

#endif
