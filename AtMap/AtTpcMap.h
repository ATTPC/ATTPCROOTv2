/*********************************************************************
 *   ATTPC Mapping Class	AtTpcMap.h			             *
 *   Author: Y. Ayyad            				     *
 *   Log: 13-02-2015 17:16 JST					     *
 *								     *
 *********************************************************************/

#ifndef ATTPCMAP_H
#define ATTPCMAP_H

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

#include "AtMap.h"

#include <cassert>
#include "TObject.h"
#include "TROOT.h"
#include "TMath.h"
#include "TCanvas.h"
#include "TH2Poly.h"
#include "TMath.h"
#include "TROOT.h"
#include "TStyle.h"
#include <map>
#include <vector>
#include <set>
#include "TDOMParser.h"
#include "TXMLNode.h"
#include "TFile.h"
#include <fstream>
#include <iostream>


class AtTpcMap : public AtMap
{

public:

  AtTpcMap();
  ~AtTpcMap();

  void Dump(); //pure virtual member
  void GenerateATTPC();//pure virtual member
  std::vector<Float_t> CalcPadCenter(Int_t PadRef);//pure virtual member
  Int_t BinToPad(Int_t binval){return binval-1;}; //pure virtual member
 
  TH2Poly* GetATTPCPlane();//virtual member
  
  Int_t  fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort);

  


  /*  friend ostream & operator << (ostream& out, const AtTpcMap& p){

      std::vector<int>::iterator it;

      for(auto it=p.PadKey.begin();it!=p.PadKey.end();it++){

      out<<"  This "<<p.PadKey[0]<<std::endl;
      }

      out<<" EN Node ID :"<<p.id<<" , ";
      out<<" EN detector segment name :"<<p.detname<<" , ";
      out<<" EN module ID :"<<p.modid<<" , ";
      out<<" EN detector ID :"<<p.detID<<" , ";
      out<<" EN VME module :"<<p.vme<<" , ";
      out<<" EN Module status :"<<p.stat<<endl;
      }*/



  ClassDef(AtTpcMap,1);

};

#endif
