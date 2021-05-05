/*********************************************************************
 *   SpecMAT Mapping Class				             *
 *   Author: B. Olaizola		        		     *
 *   Log: 2/26/2021					             *
 *								     *
 *********************************************************************/

#ifndef ATSPECMATMAP_H
#define ATSPECMATMAP_H

#include "AtMap.h"

#include <boost/multi_array.hpp>

class AtSpecMATMap : public AtMap {

public:
   AtSpecMATMap();
   ~AtSpecMATMap();

   void Dump() override;                                         // pure virtual member
   void GenerateAtTpc() override;                                // pure virtual member
   std::vector<Float_t> CalcPadCenter(Int_t PadRef) override;    // pure virtual member
   Int_t BinToPad(Int_t binval) override { return binval - 1; }; // pure virtual member

   TH2Poly *GetAtTpcPlane() override; // virtual member

   void SpecMATPadPlane();

   ClassDefOverride(AtSpecMATMap, 1);
};

#endif
