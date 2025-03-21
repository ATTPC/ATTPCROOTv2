#ifndef AT3DBRAGGCURVEHANDLER_H
#define AT3DBRAGGCURVEHANDLER_H

#include "AtPSAHitPerTBInRegion.h"

#include <Math/Point3D.h>
#include <TH1F.h>

#include <utility>
#include <vector>

class AtTrack;
class AtRawEvent;

class At3DBraggCurveHandler {
public:
   using XYZPoint = ROOT::Math::XYZPoint;

private:
   std::vector<std::pair<Double_t, Double_t>> f3DBraggCurveValues{};

   AtPSAHitPerTBInRegion *fPSA{nullptr};
   Int_t fTSSemiWidth{5};

public:
   At3DBraggCurveHandler();
   ~At3DBraggCurveHandler();

   void SetTrack(AtTrack &track);
   void SetTrack(AtTrack &track, AtRawEvent *rawEvent);

protected:
   XYZPoint FindVertex(AtTrack &track);
};

#endif
