/*********************************************************************
 *   GADGETII Mapping Class				             *
 *   Author: Y. Ayyad   ayyadlim@frib.msu.edu        		     *
 *   Log: 2/26/2021					             *
 *								     *
 *********************************************************************/

#ifndef ATGADGETIIMAP_H
#define ATGADGETIIMAP_H

#include "AtMap.h"

#include <Math/Point2Dfwd.h>
#include <Rtypes.h>

#include <unordered_map>

class TBuffer;
class TClass;
class TMemberInspector;

class AtGadgetIIMap : public AtMap {

public:
   AtGadgetIIMap();
   ~AtGadgetIIMap();

   void Dump() override;                                         // pure virtual member
   void GeneratePadPlane() override;                             // pure virtual member
   ROOT::Math::XYPoint CalcPadCenter(Int_t PadRef) override;     // pure virtual member
   Int_t BinToPad(Int_t binval) override { return binval - 1; }; // pure virtual member
   void SetBinToPadMap();

private:
   std::unordered_map<Int_t, Int_t> fBinToPadTable;
   std::unordered_map<Int_t, Int_t>::iterator fBinToPadTableIt;

   ClassDefOverride(AtGadgetIIMap, 1);
};

#endif
