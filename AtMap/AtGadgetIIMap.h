/*********************************************************************
 *   GADGETII Mapping Class				             *
 *   Author: Y. Ayyad   ayyadlim@frib.msu.edu        		     *
 *   Log: 2/26/2021					             *
 *								     *
 *********************************************************************/

#ifndef ATGADGETIIMAP_H
#define ATGADGETIIMAP_H

#include <boost/multi_array.hpp>

#include "AtMap.h"

class AtGadgetIIMap : public AtMap {

public:
   AtGadgetIIMap();
   ~AtGadgetIIMap();

   void Dump();                                         // pure virtual member
   void GenerateAtTPC();                                // pure virtual member
   std::vector<Float_t> CalcPadCenter(Int_t PadRef);    // pure virtual member
   Int_t BinToPad(Int_t binval) { return binval - 1; }; // pure virtual member

   TH2Poly *GetAtTPCPlane(); // virtual member

   ClassDef(AtGadgetIIMap, 1);
};

#endif
