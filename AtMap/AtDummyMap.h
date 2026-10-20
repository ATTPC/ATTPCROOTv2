#ifndef ATDUMMYMAP_H
#define ATDUMMYMAP_H

#include "AtMap.h"

#include <Math/Point2D.h>

/// An AtMap with no pad plane, for unit tests that need a map only to construct an object (e.g. AtPulse).
class AtDummyMap : public AtMap {
public:
   void Dump() override {}
   void GeneratePadPlane() override {}
   ROOT::Math::XYPoint CalcPadCenter(Int_t) override { return {}; }
   Int_t BinToPad(Int_t binval) override { return binval; }
};

#endif // ATDUMMYMAP_H
