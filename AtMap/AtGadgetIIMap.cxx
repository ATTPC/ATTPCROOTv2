#include "AtGadgetIIMap.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Rtypes.h>
#include <TH2Poly.h>
#include <TString.h>

#include <boost/multi_array/base.hpp>
#include <boost/multi_array/extent_gen.hpp>
#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/multi_array/subarray.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream> // IWYU pragma: keep
#include <iostream>
#include <string>
#include <vector>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";

using XYPoint = ROOT::Math::XYPoint;

AtGadgetIIMap::AtGadgetIIMap() : AtMap()
{
   AtPadCoord.resize(boost::extents[1024][4][2]);
   std::fill(AtPadCoord.data(), AtPadCoord.data() + AtPadCoord.num_elements(), 0);
   std::cout << " GADGETII Map initialized " << std::endl;
   std::cout << " GADGETII Pad Coordinates container initialized " << std::endl;
   SetBinToPadMap();
   fNumberPads = 1012;
}

AtGadgetIIMap::~AtGadgetIIMap() = default;

void AtGadgetIIMap::Dump() {}

void AtGadgetIIMap::GeneratePadPlane()
{
   if (fPadPlane) {
      LOG(error) << "Skipping generation of pad plane, it is already parsed!";
      return;
   }

   Float_t pad_size = 2.2;      // mm
   Float_t pad_spacing = 0.001; // mm

   std::vector<int> pads_per_row{18, 18, 18, 17, 17, 17, 17, 16, 16, 15, 15, 14, 13, 12, 11, 9, 7, 3};

   int pad_num = 0;

   for (auto irow = 0; irow < pads_per_row.size(); ++irow) {

      for (auto ipad = 0; ipad < pads_per_row[irow]; ++ipad) {
         AtPadCoord[ipad + pad_num][0][0] = ipad * pad_size;
         AtPadCoord[ipad + pad_num][0][1] = -irow * pad_size;
         AtPadCoord[ipad + pad_num][1][0] = ipad * pad_size + pad_size;
         AtPadCoord[ipad + pad_num][1][1] = -irow * pad_size;
         AtPadCoord[ipad + pad_num][2][0] = ipad * pad_size + pad_size;
         AtPadCoord[ipad + pad_num][2][1] = -pad_size - irow * pad_size;
         AtPadCoord[ipad + pad_num][3][0] = ipad * pad_size;
         AtPadCoord[ipad + pad_num][3][1] = -pad_size - irow * pad_size;
      }

      std::cout << " Row " << irow << " Number of pads " << pad_num << "\n";
      pad_num += pads_per_row[irow];
   }

   // Veto Pads
   AtPadCoord[pad_num][0][0] = 22.0;
   AtPadCoord[pad_num][0][1] = -34.0;
   AtPadCoord[pad_num][1][0] = 22.0;
   AtPadCoord[pad_num][1][1] = -36.0;
   AtPadCoord[pad_num][2][0] = 24.0;
   AtPadCoord[pad_num][2][1] = -36.0;
   AtPadCoord[pad_num][3][0] = 24.0;
   AtPadCoord[pad_num][3][1] = -34.0;

   pad_num += 1;

   AtPadCoord[pad_num][0][0] = 34.0;
   AtPadCoord[pad_num][0][1] = -22.0;
   AtPadCoord[pad_num][1][0] = 36.0;
   AtPadCoord[pad_num][1][1] = -22.0;
   AtPadCoord[pad_num][2][0] = 36.0;
   AtPadCoord[pad_num][2][1] = -24.0;
   AtPadCoord[pad_num][3][0] = 34.0;
   AtPadCoord[pad_num][3][1] = -24.0;

   pad_num += 1;

   for (auto irow = 0; irow < pads_per_row.size(); ++irow) {

      for (auto ipad = 0; ipad < pads_per_row[irow]; ++ipad) {
         AtPadCoord[ipad + pad_num][0][0] = -ipad * pad_size;
         AtPadCoord[ipad + pad_num][0][1] = -irow * pad_size;
         AtPadCoord[ipad + pad_num][1][0] = -ipad * pad_size - pad_size;
         AtPadCoord[ipad + pad_num][1][1] = -irow * pad_size;
         AtPadCoord[ipad + pad_num][2][0] = -ipad * pad_size - pad_size;
         AtPadCoord[ipad + pad_num][2][1] = -pad_size - irow * pad_size;
         AtPadCoord[ipad + pad_num][3][0] = -ipad * pad_size;
         AtPadCoord[ipad + pad_num][3][1] = -pad_size - irow * pad_size;
      }

      std::cout << " Row " << irow << " Number of pads " << pad_num << "\n";
      pad_num += pads_per_row[irow];
   }

   // Veto Pads
   AtPadCoord[pad_num][0][0] = -22.0;
   AtPadCoord[pad_num][0][1] = -34.0;
   AtPadCoord[pad_num][1][0] = -22.0;
   AtPadCoord[pad_num][1][1] = -36.0;
   AtPadCoord[pad_num][2][0] = -24.0;
   AtPadCoord[pad_num][2][1] = -36.0;
   AtPadCoord[pad_num][3][0] = -24.0;
   AtPadCoord[pad_num][3][1] = -34.0;

   pad_num += 1;

   AtPadCoord[pad_num][0][0] = -34.0;
   AtPadCoord[pad_num][0][1] = -22.0;
   AtPadCoord[pad_num][1][0] = -36.0;
   AtPadCoord[pad_num][1][1] = -22.0;
   AtPadCoord[pad_num][2][0] = -36.0;
   AtPadCoord[pad_num][2][1] = -24.0;
   AtPadCoord[pad_num][3][0] = -34.0;
   AtPadCoord[pad_num][3][1] = -24.0;

   pad_num += 1;

   for (auto irow = 0; irow < pads_per_row.size(); ++irow) {

      for (auto ipad = 0; ipad < pads_per_row[irow]; ++ipad) {
         AtPadCoord[ipad + pad_num][0][0] = -ipad * pad_size;
         AtPadCoord[ipad + pad_num][0][1] = irow * pad_size;
         AtPadCoord[ipad + pad_num][1][0] = -ipad * pad_size - pad_size;
         AtPadCoord[ipad + pad_num][1][1] = irow * pad_size;
         AtPadCoord[ipad + pad_num][2][0] = -ipad * pad_size - pad_size;
         AtPadCoord[ipad + pad_num][2][1] = pad_size + irow * pad_size;
         AtPadCoord[ipad + pad_num][3][0] = -ipad * pad_size;
         AtPadCoord[ipad + pad_num][3][1] = pad_size + irow * pad_size;
      }

      std::cout << " Row " << irow << " Number of pads " << pad_num << "\n";
      pad_num += pads_per_row[irow];
   }

   // Veto Pads
   AtPadCoord[pad_num][0][0] = -22.0;
   AtPadCoord[pad_num][0][1] = 34.0;
   AtPadCoord[pad_num][1][0] = -22.0;
   AtPadCoord[pad_num][1][1] = 36.0;
   AtPadCoord[pad_num][2][0] = -24.0;
   AtPadCoord[pad_num][2][1] = 36.0;
   AtPadCoord[pad_num][3][0] = -24.0;
   AtPadCoord[pad_num][3][1] = 34.0;

   pad_num += 1;

   AtPadCoord[pad_num][0][0] = -34.0;
   AtPadCoord[pad_num][0][1] = 22.0;
   AtPadCoord[pad_num][1][0] = -36.0;
   AtPadCoord[pad_num][1][1] = 22.0;
   AtPadCoord[pad_num][2][0] = -36.0;
   AtPadCoord[pad_num][2][1] = 24.0;
   AtPadCoord[pad_num][3][0] = -34.0;
   AtPadCoord[pad_num][3][1] = 24.0;

   pad_num += 1;

   for (auto irow = 0; irow < pads_per_row.size(); ++irow) {

      for (auto ipad = 0; ipad < pads_per_row[irow]; ++ipad) {
         AtPadCoord[ipad + pad_num][0][0] = ipad * pad_size;
         AtPadCoord[ipad + pad_num][0][1] = irow * pad_size;
         AtPadCoord[ipad + pad_num][1][0] = ipad * pad_size + pad_size;
         AtPadCoord[ipad + pad_num][1][1] = irow * pad_size;
         AtPadCoord[ipad + pad_num][2][0] = ipad * pad_size + pad_size;
         AtPadCoord[ipad + pad_num][2][1] = pad_size + irow * pad_size;
         AtPadCoord[ipad + pad_num][3][0] = ipad * pad_size;
         AtPadCoord[ipad + pad_num][3][1] = pad_size + irow * pad_size;
      }

      std::cout << " Row " << irow << " Number of pads " << pad_num << "\n";
      pad_num += pads_per_row[irow];
   }

   // Veto Pads
   AtPadCoord[pad_num][0][0] = 22.0;
   AtPadCoord[pad_num][0][1] = 34.0;
   AtPadCoord[pad_num][1][0] = 22.0;
   AtPadCoord[pad_num][1][1] = 36.0;
   AtPadCoord[pad_num][2][0] = 24.0;
   AtPadCoord[pad_num][2][1] = 36.0;
   AtPadCoord[pad_num][3][0] = 24.0;
   AtPadCoord[pad_num][3][1] = 34.0;

   pad_num += 1;

   AtPadCoord[pad_num][0][0] = 34.0;
   AtPadCoord[pad_num][0][1] = 22.0;
   AtPadCoord[pad_num][1][0] = 36.0;
   AtPadCoord[pad_num][1][1] = 22.0;
   AtPadCoord[pad_num][2][0] = 36.0;
   AtPadCoord[pad_num][2][1] = 24.0;
   AtPadCoord[pad_num][3][0] = 34.0;
   AtPadCoord[pad_num][3][1] = 24.0;

   pad_num += 1;

   std::cout << " Total pads " << pad_num << "\n";

   // fPadInd = pad_num;
   kIsParsed = true;

   fPadPlane = new TH2Poly(); // NOLINT
   for (auto ipad = 0; ipad < pad_num; ++ipad) {
      Double_t px[] = {AtPadCoord[ipad][0][0], AtPadCoord[ipad][1][0], AtPadCoord[ipad][2][0], AtPadCoord[ipad][3][0],
                       AtPadCoord[ipad][0][0]};
      Double_t py[] = {AtPadCoord[ipad][0][1], AtPadCoord[ipad][1][1], AtPadCoord[ipad][2][1], AtPadCoord[ipad][3][1],
                       AtPadCoord[ipad][0][1]};
      fPadPlane->AddBin(5, px, py);
   }

   fPadPlane->SetName("GADGETII_Plane");
   fPadPlane->SetTitle("GADGETII_Plane");
   fPadPlane->ChangePartition(500, 500);
}

XYPoint AtGadgetIIMap::CalcPadCenter(Int_t PadRef)
{

   if (!kIsParsed) {
      LOG(error) << " AtGadgetMap::CalcPadCenter Error : Pad plane has not been generated or parsed";
      return {-9999, -9999};
   }

   if (PadRef == -1) { // Boost multi_array crashes with a negative index
      LOG(debug) << " AtGadgetMap::CalcPadCenter Error : Pad not found";
      return {-9999, -9999};
   }

   Float_t x = (AtPadCoord[PadRef][0][0] + AtPadCoord[PadRef][1][0]) / 2.0;
   Float_t y = (AtPadCoord[PadRef][1][1] + AtPadCoord[PadRef][2][1]) / 2.0;
   return {x, y};
}

void AtGadgetIIMap::SetBinToPadMap()
{
   TString dir = getenv("VMCWORKDIR");
   TString mapFile = "GADGETII_BinToPad_08232021.txt"; // hardcoded for now
   TString mapFileWithPath = dir + "/scripts/" + mapFile;

   std::ifstream file;
   // file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

   try {

      file.open(mapFileWithPath.Data());

      while (!file.eof()) {

         std::string line;
         std::getline(file, line);

         Int_t pad = 0, bin = 0; // Energy, cross section, placeholder

         // read with default seperator (space) seperated elements
         std::istringstream isxs(line);

         isxs >> pad >> bin;

         std::cout << " Pad " << pad << " - Bin " << bin << "\n";

         fBinToPadTable.emplace(bin, pad);
      }

   } catch (...) {
      std::cout << " AtGadgetIIMap::SetBinToPadMap : Error when reading " << mapFileWithPath.Data() << "!" << cNORMAL
                << std::endl;
   }
}

/*Int_t AtGadgetIIMap::BinToPad(Int_t binval)
{



  fBinToPadTableIt = fBinToPadTable.find(binval);

   if (fBinToPadTableIt != fBinToPadTable.end()) {
      std::cout << "Element Found - ";
      std::cout << fBinToPadTableIt->first << "::" << fBinToPadTableIt->second << std::endl;
      return fBinToPadTableIt->second;
   } else {
      std::cout << "Element Not Found" << std::endl;
      return -1;
      }
      }*/

ClassImp(AtGadgetIIMap)
