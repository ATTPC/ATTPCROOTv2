/*********************************************************************
 *   SpecMAT Mapping Class				             *
 *   Author: B. Olaizola		        		     *
 *   Log: 2/26/2021					             *
 *								     *
 *********************************************************************/

#ifndef ATSPECMATMAP_H
#define ATSPECMATMAP_H

#include "AtMap.h"

#include <Math/Point2Dfwd.h>
#include <Rtypes.h>

class TBuffer;
class TClass;
class TH2Poly;
class TMemberInspector;

class AtSpecMATMap : public AtMap {

public:
   AtSpecMATMap(Int_t fNumPads = 3174);
   ~AtSpecMATMap();

   void Dump() override;                                     // pure virtual member
   void GeneratePadPlane() override;                         // pure virtual member
   ROOT::Math::XYPoint CalcPadCenter(Int_t PadRef) override; // pure virtual member
   Int_t BinToPad(Int_t binval) override { return binval; }; // pure virtual member
   TH2Poly *GetPadPlane() override;                          // virtual member

   void SpecMATPadPlane();

   ClassDefOverride(AtSpecMATMap, 1);
};

#endif
