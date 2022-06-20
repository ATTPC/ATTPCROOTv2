#ifndef ATTRACKTRANSFORMER_H
#define ATTRACKTRANSFORMER_H

#include "AtTrack.h"

#include <Rtypes.h>
#include <TObject.h>
#include <TString.h>

#include <ostream>
#include <string>
#include <vector>

class TXMLNode;
class TBuffer;
class TClass;
class TMemberInspector;

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
