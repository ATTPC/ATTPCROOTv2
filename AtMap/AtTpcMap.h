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

class AtTpcMap : public AtMap {

public:
   AtTpcMap();
   ~AtTpcMap();

   virtual void Dump() override;
   virtual void GenerateAtTpc() override;
   virtual std::vector<Float_t> CalcPadCenter(Int_t PadRef) override;
   virtual TH2Poly *GetAtTpcPlane() override;
   virtual Int_t BinToPad(Int_t binval) override { return binval - 1; };

   Int_t fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort);

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

   ClassDef(AtTpcMap, 1);
};

#endif
