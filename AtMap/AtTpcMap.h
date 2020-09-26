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


class AtTpcMap : public TNamed
{

public:

  AtTpcMap();
  ~AtTpcMap();

#ifndef __CINT__
  typedef boost::multi_array<double,3> multiarray;
  typedef multiarray::index index;
  multiarray AtPadCoord;
  virtual multiarray GetPadCoordArr(){return AtPadCoord;}
  virtual std::vector<Float_t> CalcPadCenter(Int_t PadRef);
  //multiarray *GetPadCoord(){return fAtPadCoordPtr=&AtPadCoord;}
  //multiarray *fAtPadCoordPtr;
#endif //__CINT__



  virtual void Initialize();
  virtual void Dump();

  virtual void GenerateATTPC();


  virtual Int_t  fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort);
  inline void SetGUIMode(){kGUIMode=1;}
  inline void SetDebugMode(){kDebug=1;}
  Bool_t ParseXMLMap(Char_t const *xmlfile);
  void ParseMapList(TXMLNode *node);
  void ParseATTPCMap(TXMLNode *node);
  Bool_t ParseInhibitMap(TString inimap, TString lowgmap, TString xtalkmap);
  Bool_t DumpATTPCMap();
  Int_t  GetPadNum(std::vector<int> PadRef);
  Bool_t GetIsInhibited(Int_t PadNum);
  std::vector<int> GetPadRef(int padNum);

// This is to use polymorphism of the derived class for Prototype. We need the definition here as well for pointers of the base class to the derived ie: TAtTpcMap *foo = new TAtpProtoMap()
  virtual Bool_t SetGeoFile(TString geofile){std::cout<<" = AtTpcMap Warning: SetGeoFile method from Base invoked"<<std::endl;return kFALSE;} // TODO This is a non-pure virtual function overriden by the method in AtTpcProtoMap. Make it pure by creating another derived class for ATTPC
  virtual TH2Poly* GetATTPCPlane(TString TH2Poly_name){std::cout<<" = AtTpcMap Warning: GetATTPCPlane overloaded method from Base invoked"<<std::endl;return NULL;}
  virtual Bool_t SetProtoMap(TString file){std::cout<<" = AtTpcMap Warning: SetProtoMap method from Base invoked"<<std::endl;return kFALSE;}
  virtual Int_t BinToPad(Int_t binval){std::cout<<" = AtTpcMap Warning: SetProtoMap method from Base invoked"<<std::endl;return binval-1;}




  virtual TH2Poly* GetATTPCPlane();
  Int_t fPadInd;
  Bool_t kIsParsed;
  Bool_t kGUIMode;
  Bool_t kDebug;



  std::map<std::vector<int>,int> ATTPCPadMap;
  std::map<int,std::vector<int>> ATTPCPadMapInverse;
  std::vector<int> PadKey;
  std::set<Int_t> fIniPads;



  TCanvas *cATTPCPlane;
  TH2Poly *hPlane; //TODO Only allowed by C+11, change the initialization to the cxx

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
