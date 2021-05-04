/*********************************************************************
 *   SpecMAT Mapping Class				             *
 *   Author: B. Olaizola      		     *
 *   Log: 2/26/2021					             *
 *								     *
 *********************************************************************/

#ifndef ATSPECMATMAP_H
#define ATSPECMATMAP_H


#include <boost/multi_array.hpp>


#include "AtMap.h"



class AtSpecMATMap : public AtMap
{

public:

  AtSpecMATMap();
  ~AtSpecMATMap();

  void Dump(); //pure virtual member
  void GenerateATTPC();//pure virtual member
  std::vector<Float_t> CalcPadCenter(Int_t PadRef);//pure virtual member
  Int_t BinToPad(Int_t binval){return binval-1;}; //pure virtual member

  TH2Poly* GetATTPCPlane();//virtual member

ClassDef(AtSpecMATMap,1);

};
 
#endif
