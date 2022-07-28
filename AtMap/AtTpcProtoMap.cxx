/*********************************************************************
 *   ATTPC Mapping Class	AtTpcProtoMap.cxx			             *
 *   Author: Y. Ayyad            				     *
 *   Log: 31-03-2015 11:56 JST					     *
 *								     *
 *********************************************************************/

#include "AtTpcProtoMap.h"

#include <Math/Point2D.h>
#include <Rtypes.h>
#include <TCollection.h>
#include <TDirectory.h>
#include <TFile.h>
#include <TH2Poly.h>
#include <TKey.h>
#include <TMultiGraph.h>
#include <TObject.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <utility>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
using XYPoint = ROOT::Math::XYPoint;

ClassImp(AtTpcProtoMap);

AtTpcProtoMap::AtTpcProtoMap() : AtMap() {}

Bool_t AtTpcProtoMap::SetGeoFile(TString geofile)
{

   TString dir = getenv("VMCWORKDIR");
   TString geodir = dir + "/geometry/" + geofile;
   f = new TFile(geodir.Data()); // NOLINT

   if (f->IsZombie()) {
      std::cout << cRED << " AtTPC Proto Map : No geometry file found! Check VMCWORKDIR variable. Exiting... "
                << cNORMAL << std::endl; // TODO Not working!!!
      delete f;
      return kFALSE;
   }
   std::cout << cGREEN << " AtTPC Proto Map : Prototype geometry found in : " << geodir.Data() << cNORMAL << std::endl;
   kIsFileSet = kTRUE;
   return kTRUE;
}

void AtTpcProtoMap::GeneratePadPlane()
{

   if (f->IsZombie()) {
      std::cout
         << " AtTPC Proto Map : No geometry file found! Please set the geometry file first via SetGeoFile method "
         << std::endl;
      return;
   }

   std::cout << " AtTPC Proto Map : Generating the map geometry of the AtTPC Prototype " << std::endl;
   TMultiGraph *mg = nullptr;
   TKey *key = nullptr;
   TIter nextkey(gDirectory->GetListOfKeys());
   while ((key = dynamic_cast<TKey *>(nextkey()))) {
      auto *obj = dynamic_cast<TMultiGraph *>(key->ReadObj());
      if (obj != nullptr)
         fPadPlane->AddBin(obj);
      auto padPlane = dynamic_cast<TH2Poly *>(key->ReadObj());
      if (padPlane != nullptr) {
         if (fPadPlane != nullptr)
            delete fPadPlane;
         fPadPlane = dynamic_cast<TH2Poly *>(padPlane->Clone());
         break;
      }
      /*std::cout << "Using key: " << key << " " << obj << std::endl;
      if (obj->InheritsFrom("TMultiGraph")) {
         mg = (TMultiGraph *)obj;
         bin = fPadPlane->AddBin(mg);
         // std::cout<<bin<<std::endl;
      }
      */
   }

   kIsGenerated = kTRUE;
}

TH2Poly *AtTpcProtoMap::GetPadPlane()
{
   // This method must be called after GenerateAtTPC()

   if (!kIsFileSet) {
      std::cout
         << " AtTPC Proto Map : No geometry file found! Please set the geometry file first via the SetGeoFile method "
         << std::endl;
      return nullptr;
   }

   if (!kIsGenerated) {
      std::cout
         << "  AtTPC Proto Map : Pad plane has not been generated. Please generate it via the GenerateAtTPC method "
         << std::endl;
      return nullptr;
   }

   if (kGUIMode)
      drawPadPlane();

   return fPadPlane;
}

TH2Poly *AtTpcProtoMap::GetAtTpcPlane(TString TH2Poly_name)
{
   // This is a stand alone method
   if (f->IsZombie()) {
      std::cout
         << " AtTPC Proto Map : No geometry file found! Please set the geometry file first via SetGeoFile method "
         << std::endl;
      return nullptr;
   }
   fPadPlane = dynamic_cast<TH2Poly *>(f->Get(TH2Poly_name.Data()));
   return fPadPlane;
}

XYPoint AtTpcProtoMap::CalcPadCenter(Int_t PadRef)
{

   if (!kIsProtoMapSet) {
      std::cout << " AtTPC Proto Map : No map file for prototype found! Please set the geometry file first via the "
                   "SetProtoMap method "
                << std::endl;
      return {-9999, -9999};
   }

   if (f->IsZombie()) {
      std::cout
         << " AtTPC Proto Map : No geometry file found! Please set the geometry file first via the SetGeoFile method "
         << std::endl;
      return {-9999, -9999};
   }

   if (PadRef != -1) { // Boost multi_array crashes with a negative index
      auto its = ProtoGeoMap.find(PadRef);

      Int_t kIs = Int_t(ProtoGeoMap.find(PadRef) == ProtoGeoMap.end());
      if (kIs) {
         if (kDebug)
            std::cerr << " AtTpcProtoMap::CalcPadCenter - Pad  not found - CoboID : " << PadRef << std::endl;
         return {-9999, -9999};
      }

      auto padCenter = (*its).second;
      return {padCenter[0], padCenter[1]};

   } else {

      if (kDebug)
         std::cout << " AtTpcProtoMap::CalcPadCenter Error : Pad not found" << std::endl;
      return {-9999, -9999};
   }
}

Bool_t AtTpcProtoMap::SetProtoMap(TString file)
{

   /* Sets a map for the prototype containing the Pad Number*/
   TString PadName;
   Int_t PadNum = -1;
   Float_t pad_xcoord = -1;
   Float_t pad_ycoord = -1;
   Int_t bin_num = -1;
   std::vector<Float_t> PadCoord;
   PadCoord.reserve(2);
   auto InProtoMap = std::make_unique<std::ifstream>(file.Data());

   if (InProtoMap->fail()) {
      std::cout << " = AtTpcProtoMap::SetProtoMap : No Prototype Map file found! Please, check the path. Current :"
                << file.Data() << std::endl;
      return -1;
   } else
      kIsProtoMapSet = kTRUE;

   while (!InProtoMap->eof()) {

      *InProtoMap >> PadNum >> PadName >> pad_xcoord >> pad_ycoord >> bin_num;
      PadCoord.push_back(pad_xcoord);
      PadCoord.push_back(pad_ycoord);
      if (kDebug)
         if (!PadName.IsNull())
            std::cout << " PadNum : " << PadNum << " - PadName : " << PadName.Data()
                      << " - Pad X coord : " << pad_xcoord << " - Pad Y coord : " << pad_ycoord
                      << " - Bin number : " << bin_num << std::endl;
      ProtoGeoMap.insert(std::pair<Int_t, std::vector<Float_t>>(PadNum, PadCoord));
      ProtoBinMap.insert(std::pair<Int_t, Int_t>(bin_num, PadNum));
      PadCoord.clear();
   }

   return kTRUE;
}

Int_t AtTpcProtoMap::BinToPad(Int_t binval)
{

   if (!kIsProtoMapSet) {
      std::cout << " = AtTpcProtoMap::BinToPad : No Prototype Map file found! Please, use the SetProtoMap method first."
                << std::endl;
      return -1;
   }

   auto its = ProtoBinMap.find(binval);
   Int_t padval = (*its).second;
   Int_t kIs = int(ProtoBinMap.find(binval) == ProtoBinMap.end());
   if (kIs) {
      if (kDebug)
         std::cerr << " = AtTpcProtoMap::BinToPad - Bin not found : " << binval << std::endl;
      return -1;
   } else if (binval > 2014 || binval < 0) {

      std::cout
         << " = AtTpcProtoMap::BinToPad - Warning: Bin value out of expected boundaries for prototype bin mapping : "
         << binval << std::endl;
   }

   return padval;
}

void AtTpcProtoMap::Dump() {}
