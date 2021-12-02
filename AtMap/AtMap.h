
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

// The definition of this struct, and the operator overloads have to
// be before AtMap where an unordered_map using this as a key is
// instatiated.
struct PadReference {
   Int_t cobo;
   Int_t asad;
   Int_t aget;
   Int_t ch;
};
bool operator<(const PadReference &l, const PadReference &r);
bool operator==(const PadReference &l, const PadReference &r);
namespace std {
template <>
struct hash<PadReference> {
   inline size_t operator()(const PadReference &x) const
   {
      return x.ch + x.aget * 100 + x.asad * 10000 + x.cobo * 1000000;
   }
};
} // namespace std

class AtMap : public TNamed {

protected:
   typedef boost::multi_array<double, 3> multiarray;
   typedef multiarray::index index;

   multiarray AtPadCoord;
   multiarray *fAtPadCoordPtr;
   Int_t fPadInd;
   Bool_t kIsParsed;
   Bool_t kGUIMode;
   Bool_t kDebug;
   std::set<Int_t> fIniPads;
   TCanvas *cAtTPCPlane;
   TH2Poly *hPlane;
   std::unordered_map<PadReference, int> AtTPCPadMap;
   std::map<int, PadReference> AtTPCPadMapInverse;
   std::unordered_map<PadReference, std::string> fAuxPadMap;
   std::map<int, int> AtTPCPadSize;

public:
   AtMap();
   ~AtMap();

   virtual void Dump() = 0;
   virtual void GenerateAtTpc() = 0;
   virtual std::vector<Float_t> CalcPadCenter(Int_t PadRef) = 0;
   virtual TH2Poly *GetAtTpcPlane() = 0;
   virtual Int_t BinToPad(Int_t binval) = 0;

   Int_t GetPadNum(const PadReference &PadRef) const;
   multiarray GetPadCoordArr() { return AtPadCoord; }
   multiarray *GetPadCoord() { return fAtPadCoordPtr = &AtPadCoord; }

   Bool_t ParseXMLMap(Char_t const *xmlfile);
   void ParseMapList(TXMLNode *node);
   void ParseAtTPCMap(TXMLNode *node);
   Bool_t DumpAtTPCMap();
   PadReference GetPadRef(int padNum) const;
   bool AddAuxPad(const PadReference &ref, std::string auxName);
   bool IsAuxPad(const PadReference &ref) const;
   std::string GetAuxName(const PadReference &ref) const;

   inline void SetGUIMode() { kGUIMode = 1; }
   inline void SetDebugMode() { kDebug = 1; }
   Bool_t ParseInhibitMap(TString inimap, TString lowgmap, TString xtalkmap);
   Bool_t GetIsInhibited(Int_t PadNum);
   Int_t GetPadSize(int padNum);

   ClassDefOverride(AtMap, 2);
};

#endif
