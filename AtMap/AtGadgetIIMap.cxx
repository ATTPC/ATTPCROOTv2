#include "AtGadgetIIMap.h"

#include "TCanvas.h"
#include "TH2Poly.h"
#include "TStyle.h"

#include <iostream>
#include <fstream>
#include <cassert>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"

AtGadgetIIMap::AtGadgetIIMap()
{
   AtPadCoord.resize(boost::extents[1024][4][2]);
   kIsParsed = 0;
   kGUIMode = 0;
   kDebug = 0;
   std::fill(AtPadCoord.data(), AtPadCoord.data() + AtPadCoord.num_elements(), 0);
   std::cout << " GADGETII Map initialized " << std::endl;
   std::cout << " GADGETII Pad Coordinates container initialized " << std::endl;
   SetBinToPadMap();
   fPadInd = 0;
   fNumberPads = 1012;
   fIniPads.clear();
   hPlane = new TH2Poly();
}

AtGadgetIIMap::~AtGadgetIIMap() {}

void AtGadgetIIMap::Dump() {}

void AtGadgetIIMap::GenerateAtTpc()
{
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

   fPadInd = pad_num;

   for (auto ipad = 0; ipad < fPadInd; ++ipad) {
      Double_t px[] = {AtPadCoord[ipad][0][0], AtPadCoord[ipad][1][0], AtPadCoord[ipad][2][0], AtPadCoord[ipad][3][0],
                       AtPadCoord[ipad][0][0]};
      Double_t py[] = {AtPadCoord[ipad][0][1], AtPadCoord[ipad][1][1], AtPadCoord[ipad][2][1], AtPadCoord[ipad][3][1],
                       AtPadCoord[ipad][0][1]};
      hPlane->AddBin(5, px, py);
   }

   // for(auto isec = 0; isec < 2; ++isec){
   //	   for (auto ipad = 0; ipad < fPadInd; ++ipad) { // todo: Check total number of pads
   //
   //	      Double_t px[] = {AtPadCoord[ipad][0][0]*TMath::Power(-1,isec), AtPadCoord[ipad][1][0]*TMath::Power(-1,isec),
   // AtPadCoord[ipad][2][0]*TMath::Power(-1,isec), AtPadCoord[ipad][3][0]*TMath::Power(-1,isec),
   //			       AtPadCoord[ipad][0][0]*TMath::Power(-1,isec)};
   //	      Double_t py[] = {AtPadCoord[ipad][0][1]*TMath::Power(-1,isec), AtPadCoord[ipad][1][1]*TMath::Power(-1,isec),
   // AtPadCoord[ipad][2][1]*TMath::Power(-1,isec), AtPadCoord[ipad][3][1]*TMath::Power(-1,isec),
   //			       AtPadCoord[ipad][0][1]*TMath::Power(-1,isec)};
   //	      hPlane->AddBin(5, px, py);
   //	   }
   // }
   //
   //
   //		for(auto isec = 1; isec < 3; ++isec){
   //			   for (auto ipad = 0; ipad < fPadInd; ++ipad) { // todo: Check total number of pads
   //
   //			      Double_t px[] = {AtPadCoord[ipad][0][0]*TMath::Power(-1,isec),
   // AtPadCoord[ipad][1][0]*TMath::Power(-1,isec), AtPadCoord[ipad][2][0]*TMath::Power(-1,isec),
   // AtPadCoord[ipad][3][0]*TMath::Power(-1,isec), 					       AtPadCoord[ipad][0][0]*TMath::Power(-1,isec)};
   // Double_t py[] = {AtPadCoord[ipad][0][1]*TMath::Power(-1,isec+1), AtPadCoord[ipad][1][1]*TMath::Power(-1,isec+1),
   // AtPadCoord[ipad][2][1]*TMath::Power(-1,isec+1), AtPadCoord[ipad][3][1]*TMath::Power(-1,isec+1),
   //					       AtPadCoord[ipad][0][1]*TMath::Power(-1,isec+1)};
   //			      hPlane->AddBin(5, px, py);
   //			   }
   //		}
   //
}

std::vector<Float_t> AtGadgetIIMap::CalcPadCenter(Int_t PadRef)
{

   std::vector<Float_t> PadCenter = {-9999, -9999};
   PadCenter.reserve(2);

   if (!fPadInd || !kIsParsed) {

      std::cout << " AtTpcMap::CalcPadCenter Error : Pad plane has not been generated or parsed " << std::endl;
      return PadCenter;
   }

   if (PadRef != -1) { // Boost multi_array crashes with a negative index

      Float_t x = (AtPadCoord[PadRef][0][0] + AtPadCoord[PadRef][1][0]) / 2.0;
      PadCenter[0] = x;
      Float_t y = (AtPadCoord[PadRef][1][1] + AtPadCoord[PadRef][2][1]) / 2.0;
      PadCenter[1] = y;
      return PadCenter;

   } else {

      if (kDebug)
         std::cout << " AtTpcMap::CalcPadCenter Error : Pad not found" << std::endl;
      return PadCenter;
   }
}

TH2Poly *AtGadgetIIMap::GetAtTpcPlane()
{

   if (fPadInd == 0) {

      std::cout << " AtGadgetIIMap::GetAtTPCPlane Error : Pad plane has not been generated - Exiting... " << std::endl;

      return NULL;
   }

   hPlane->SetName("GADGETII_Plane");
   hPlane->SetTitle("GADGETII_Plane");
   hPlane->ChangePartition(500, 500);

   if (kGUIMode) {
      cAtTPCPlane = new TCanvas("cAtTPCPlane", "cAtTPCPlane", 1000, 1000);
      gStyle->SetPalette(1);
      hPlane->Draw("COL L");
   }

   return hPlane;
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
