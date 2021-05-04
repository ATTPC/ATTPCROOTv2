#include "AtGadgetIIMap.h"

#include <iostream>
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
   fPadInd = 0;
   PadKey.clear();
   fIniPads.clear();
   hPlane = new TH2Poly();
}

AtGadgetIIMap::~AtGadgetIIMap() {}

void AtGadgetIIMap::Dump() {}

void AtGadgetIIMap::GenerateAtTpc()
{
   Float_t pad_size = 2.0;      // mm
   Float_t pad_spacing = 0.001; // mm

   std::vector<int> pads_per_row{18, 18, 18, 17, 17, 17, 17, 16, 16, 15, 15, 14, 13, 12, 11, 9, 7, 3};

   int pad_num = 0;

   for (auto irow = 0; irow < pads_per_row.size(); ++irow) {

      pad_num += pads_per_row[irow];

      std::cout << " Row " << irow << " Number of pads " << pad_num << "\n";

      for (auto ipad = 0; ipad < pads_per_row[irow]; ++ipad) {
         AtPadCoord[ipad + pad_num][0][0] = ipad * 2;
         AtPadCoord[ipad + pad_num][0][1] = -irow * 2;
         AtPadCoord[ipad + pad_num][1][0] = ipad * 2 + 2;
         AtPadCoord[ipad + pad_num][1][1] = -irow * 2;
         AtPadCoord[ipad + pad_num][2][0] = ipad * 2 + 2;
         AtPadCoord[ipad + pad_num][2][1] = -2 - irow * 2;
         AtPadCoord[ipad + pad_num][3][0] = ipad * 2;
         AtPadCoord[ipad + pad_num][3][1] = -2 - irow * 2;
      }
   }

   fPadInd = pad_num;

	for(auto isec = 0; isec < 2; ++isec){
		   for (auto ipad = 0; ipad < fPadInd; ++ipad) { // todo: Check total number of pads

		      Double_t px[] = {AtPadCoord[ipad][0][0]*TMath::Power(-1,isec), AtPadCoord[ipad][1][0]*TMath::Power(-1,isec), AtPadCoord[ipad][2][0]*TMath::Power(-1,isec), AtPadCoord[ipad][3][0]*TMath::Power(-1,isec),
				       AtPadCoord[ipad][0][0]*TMath::Power(-1,isec)};
		      Double_t py[] = {AtPadCoord[ipad][0][1]*TMath::Power(-1,isec), AtPadCoord[ipad][1][1]*TMath::Power(-1,isec), AtPadCoord[ipad][2][1]*TMath::Power(-1,isec), AtPadCoord[ipad][3][1]*TMath::Power(-1,isec),
				       AtPadCoord[ipad][0][1]*TMath::Power(-1,isec)};
		      hPlane->AddBin(5, px, py);
		   } 
	}


			for(auto isec = 1; isec < 3; ++isec){
				   for (auto ipad = 0; ipad < fPadInd; ++ipad) { // todo: Check total number of pads

				      Double_t px[] = {AtPadCoord[ipad][0][0]*TMath::Power(-1,isec), AtPadCoord[ipad][1][0]*TMath::Power(-1,isec), AtPadCoord[ipad][2][0]*TMath::Power(-1,isec), AtPadCoord[ipad][3][0]*TMath::Power(-1,isec),
						       AtPadCoord[ipad][0][0]*TMath::Power(-1,isec)};
				      Double_t py[] = {AtPadCoord[ipad][0][1]*TMath::Power(-1,isec+1), AtPadCoord[ipad][1][1]*TMath::Power(-1,isec+1), AtPadCoord[ipad][2][1]*TMath::Power(-1,isec+1), AtPadCoord[ipad][3][1]*TMath::Power(-1,isec+1),
						       AtPadCoord[ipad][0][1]*TMath::Power(-1,isec+1)};
				      hPlane->AddBin(5, px, py);
				   } 
			}






}

std::vector<Float_t> AtGadgetIIMap::CalcPadCenter(Int_t PadRef) {}

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

ClassImp(AtGadgetIIMap)
