#include "AtGadgetIIMap.h"

#include <iostream>
#include <cassert>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"


AtGadgetIIMap::AtGadgetIIMap()
{
  AtPadCoord.resize(boost::extents[1024][4][2]);
  kIsParsed=0;
  kGUIMode=0;
  kDebug=0;
  std::fill( AtPadCoord.data(), AtPadCoord.data()+AtPadCoord.num_elements() , 0);
  std::cout<<" GADGETII Map initialized "<<std::endl;
  std::cout<<" GADGETII Pad Coordinates container initialized "<<std::endl;
  fPadInd = 0;
  PadKey.clear();
  fIniPads.clear();
  hPlane = new TH2Poly();
}

AtGadgetIIMap::~AtGadgetIIMap()
{
  
}

void AtGadgetIIMap::Dump(){

  

}

void AtGadgetIIMap::GenerateATTPC(){


}

std::vector<Float_t> AtGadgetIIMap::CalcPadCenter(Int_t PadRef){

  

}

TH2Poly* AtGadgetIIMap::GetATTPCPlane(){




}


ClassImp(AtGadgetIIMap)



