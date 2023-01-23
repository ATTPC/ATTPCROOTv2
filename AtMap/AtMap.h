/*********************************************************************
 *   Base class for Active Targets AtMap.h			     *
 *   Author: Y. Ayyad ayyadlim@frib.msu.edu            	             *
 *   Log: 2/22/2021 					             *
 *								     *
 *********************************************************************/

#ifndef ATMAP_H
#define ATMAP_H

#include "AtPadReference.h"

#include <Math/Point2Dfwd.h>
#include <Rtypes.h>
#include <TNamed.h>
#include <TString.h>

#include <boost/multi_array.hpp>

#include <functional>
#include <iosfwd>
#include <map>
#include <string>
#include <unordered_map>

class TH2Poly;
class TXMLNode;
class TBuffer;
class TCanvas;
class TClass;
class TMemberInspector;

class AtMap : public TNamed {
public:
   enum class InhibitType; // forward declare of enum

protected:
   typedef boost::multi_array<double, 3> multiarray;
   typedef multiarray::index index;

   multiarray AtPadCoord;
   multiarray *fAtPadCoordPtr{};
   Bool_t kIsParsed = false; //< True if the input file is parsed
   Bool_t kDebug = false;
   std::map<Int_t, AtMap::InhibitType> fIniPads;
   TCanvas *fPadPlaneCanvas{}; // Raw pointer because owned by gROOT
   TH2Poly *fPadPlane;         // Raw pointer because owned by gDirectory
   UInt_t fNumberPads{};

   std::unordered_map<AtPadReference, int> fPadMap;
   std::map<int, AtPadReference> fPadMapInverse;
   std::unordered_map<AtPadReference, std::string> fAuxPadMap;
   std::map<int, int> fPadSizeMap;

   void inhibitPad(Int_t padNum, AtMap::InhibitType type);
   void drawPadPlane();

public:
   AtMap();
   ~AtMap() = default;

   virtual void Dump() = 0;
   /**
    * Virtual function that creates the TH2Poly that stores the pad plane geometry
    */
   virtual void GeneratePadPlane() = 0;
   virtual ROOT::Math::XYPoint CalcPadCenter(Int_t PadRef) = 0; // units mm

   /// Assumes it will generate the pad plane if it hasn't been generated already
   /// returns a clone of the internal pad plane which is owned by ROOT
   TH2Poly *GetPadPlane();
   virtual Int_t BinToPad(Int_t binval) = 0;

   UInt_t GetNumPads() const { return fNumberPads; }

   Int_t GetPadNum(const AtPadReference &PadRef) const;
   multiarray GetPadCoordArr() { return AtPadCoord; }
   multiarray *GetPadCoord() { return fAtPadCoordPtr = &AtPadCoord; }

   Bool_t ParseXMLMap(Char_t const *xmlfile);
   void ParseMapList(TXMLNode *node);
   void ParseAtTPCMap(TXMLNode *node);
   Bool_t DumpAtTPCMap();
   AtPadReference GetPadRef(int padNum) const;
   bool AddAuxPad(const AtPadReference &ref, std::string auxName);
   bool IsAuxPad(const AtPadReference &ref) const;
   bool IsFPNchannel(const AtPadReference &ref) const;
   AtPadReference GetNearestFPN(int padNum) const;
   AtPadReference GetNearestFPN(const AtPadReference &ref) const;

   std::string GetAuxName(const AtPadReference &ref) const;

   inline void SetDebugMode(Bool_t flag = true) { kDebug = flag; }
   Bool_t ParseInhibitMap(TString inimap, AtMap::InhibitType type);
   AtMap::InhibitType IsInhibited(Int_t PadNum);
   Int_t GetPadSize(int padNum);

#pragma GCC diagnostic push
   // Ignore shadow warning when we shadow ROOT's global GuiTypes enum
#pragma GCC diagnostic ignored "-Wshadow"

   // The higher the number, the higher the priority
   // i.e. Adding a pad to the inhibit map with kTotal and kLowGain
   // will inhibit the pad. kLowGain and kXTalk will be kXTalk
   enum class InhibitType { kNone = 0, kLowGain = 1, kXTalk = 2, kTotal = 3 };
#pragma GCC diagnostic pop

   ClassDefOverride(AtMap, 5);
};

std::ostream &operator<<(std::ostream &os, const AtMap::InhibitType &t);
#endif
