/*********************************************************************
 *   Base class for Active Targets AtMap.h			     *
 *   Author: Y. Ayyad ayyadlim@frib.msu.edu            	             *
 *   Log: 2/22/2021 					             *
 *								     *
 *********************************************************************/

#ifndef ATMAP_H
#define ATMAP_H

#include <boost/multi_array.hpp>

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

class AtMap : public TNamed {

public:
   AtMap();
   ~AtMap();

   typedef boost::multi_array<double, 3> multiarray;
   typedef multiarray::index index;
   struct PadReference {
      Int_t cobo;
      Int_t asad;
      Int_t aget;
      Int_t ch;
   };

   virtual void Dump() = 0;
   virtual void GenerateAtTpc() = 0;
   virtual std::vector<Float_t> CalcPadCenter(Int_t PadRef) = 0;
   virtual TH2Poly *GetAtTpcPlane() = 0;
   virtual Int_t BinToPad(Int_t binval) = 0;

   Int_t GetPadNum(const AtMap::PadReference &PadRef) const;
   multiarray GetPadCoordArr() { return AtPadCoord; }
   multiarray *GetPadCoord() { return fAtPadCoordPtr = &AtPadCoord; }

   Bool_t ParseXMLMap(Char_t const *xmlfile);
   void ParseMapList(TXMLNode *node);
   void ParseAtTPCMap(TXMLNode *node);
   Bool_t DumpAtTPCMap();
   AtMap::PadReference GetPadRef(int padNum) const;
   // PadReference GetPadRef(const AtPad &pad) const {return GetPadRef(pad.GetPadNum())};

   inline void SetGUIMode() { kGUIMode = 1; }
   inline void SetDebugMode() { kDebug = 1; }
   Bool_t ParseInhibitMap(TString inimap, TString lowgmap, TString xtalkmap);
   Bool_t GetIsInhibited(Int_t PadNum);
   Int_t GetPadSize(int padNum);

   multiarray AtPadCoord;
   multiarray *fAtPadCoordPtr;
   Int_t fPadInd;
   Bool_t kIsParsed;
   Bool_t kGUIMode;
   Bool_t kDebug;
   std::set<Int_t> fIniPads;
   TCanvas *cAtTPCPlane;
   TH2Poly *hPlane;
   std::map<AtMap::PadReference, int> AtTPCPadMap;
   std::map<int, AtMap::PadReference> AtTPCPadMapInverse;
   std::map<int, int> AtTPCPadSize;

private:
   ClassDefOverride(AtMap, 1);
};

bool operator<(const AtMap::PadReference &l, const AtMap::PadReference &r);

#endif
