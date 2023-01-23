/*********************************************************************
 *   ATTPC Mapping Class	AtTpcMap.h			             *
 *   Author: Y. Ayyad            				     *
 *   Log: 13-02-2015 17:16 JST					     *
 *								     *
 *********************************************************************/

#ifndef ATTPCMAP_H
#define ATTPCMAP_H
#include "AtMap.h"

#include <Math/Point2Dfwd.h>
#include <Rtypes.h>

class TBuffer;
class TClass;
class TMemberInspector;

class AtTpcMap : public AtMap {

public:
   AtTpcMap();
   ~AtTpcMap();

   virtual void Dump() override;
   virtual void GeneratePadPlane() override;
   virtual ROOT::Math::XYPoint CalcPadCenter(Int_t PadRef) override;
   virtual Int_t BinToPad(Int_t binval) override { return binval - 1; };

   ClassDefOverride(AtTpcMap, 1);

protected:
   Int_t fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort);
};

#endif
