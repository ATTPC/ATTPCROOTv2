
/*********************************************************************
 *   Base class for Active Targets AtMap.h			     *
 *   Author: Y. Ayyad ayyadlim@frib.msu.edu            	             *
 *   Log: 2/22/2021 					             *
 *								     *
 *********************************************************************/

#ifndef ATMAP_H
#define ATMAP_H

#include <boost/multi_array.hpp>
#include <Math/Point2Dfwd.h>
#include <Rtypes.h>
#include <RtypesCore.h>
#include <TNamed.h>
#include <TString.h>
#include <unordered_map>
#include <map>
#include <iosfwd>
#include <string>

#include "PadReference.h"

class TH2Poly;
class TXMLNode;
class TBuffer;
class TCanvas;
class TClass;
class TMemberInspector;

class AtMap : public TNamed {
public:
   enum InhibitType : char; // forward declare of enum

protected:
   typedef boost::multi_array<double, 3> multiarray;
   typedef multiarray::index index;

   multiarray AtPadCoord;
   multiarray *fAtPadCoordPtr;
   Bool_t kIsParsed = false;
   Bool_t kGUIMode = false;
   Bool_t kDebug = false;
   std::map<Int_t, AtMap::InhibitType> fIniPads;
   TCanvas *fPadPlaneCanvas; // Raw pointer because owned by gROOT
   TH2Poly *fPadPlane;       // Raw pointer because owned by gDirectory
   UInt_t fNumberPads;

   std::unordered_map<PadReference, int> fPadMap;
   std::map<int, PadReference> fPadMapInverse;
   std::unordered_map<PadReference, std::string> fAuxPadMap;
   std::map<int, int> fPadSizeMap;

   void inhibitPad(Int_t padNum, AtMap::InhibitType type);
   void drawPadPlane();

public:
   AtMap();
   ~AtMap() = default;

   virtual void Dump() = 0;
   virtual void GeneratePadPlane() = 0;
   virtual ROOT::Math::XYPoint CalcPadCenter(Int_t PadRef) = 0; // units mm
   virtual TH2Poly *GetPadPlane() = 0;
   virtual Int_t BinToPad(Int_t binval) = 0;

   UInt_t GetNumPads() const { return fNumberPads; }

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
   inline void SetDebugMode(Bool_t flag = true) { kDebug = flag; }
   Bool_t ParseInhibitMap(TString inimap, AtMap::InhibitType type);
   AtMap::InhibitType IsInhibited(Int_t PadNum);
   Int_t GetPadSize(int padNum);

   // The higher the number, the higher the priority
   // i.e. Adding a pad to the inhibit map with kTotal and kLowGain
   // will inhibit the pad. kLowGain and kXTalk will be kXTalk
   enum InhibitType : char { kNone = 0, kLowGain = 1, kXTalk = 2, kTotal = 3 };

   ClassDefOverride(AtMap, 5);
};

std::ostream &operator<<(std::ostream &os, const AtMap::InhibitType &t);
#endif
