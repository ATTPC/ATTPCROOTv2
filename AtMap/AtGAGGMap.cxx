/*********************************************************************
 *   ATGAGG Mapping Class	AtGAGGMap.cxx			                         *
 *   Author: T. J. Gray            				                           *
 *   Log: 05-05-2026                                   					     *
 *								                                                   *
 *********************************************************************/

#include "AtGAGGMap.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Rtypes.h>
#include <TH2Poly.h>
#include <TMath.h>
#include <TMathBase.h>
#include <TXMLDocument.h>
#include <TXMLNode.h>

#include <boost/multi_array/base.hpp>
#include <boost/multi_array/extent_gen.hpp>
#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/multi_array/subarray.hpp>

#include <algorithm>
#include <cmath>
#include <fstream> // IWYU pragma: keep
#include <iostream>
#include <vector>

#undef BOOST_MULTI_ARRAY_NO_GENERATORS
#define BOOST_MULTI_ARRAY_NO_GENERATORS

using std::cout;
using std::endl;
using XYPoint = ROOT::Math::XYPoint;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";

AtGAGGMap::AtGAGGMap() : AtMap()
{
   AtPadCoord.resize(boost::extents[40][3][2]);
   std::fill(AtPadCoord.data(), AtPadCoord.data() + AtPadCoord.num_elements(), 0);
   std::cout << " ATGAGG Map initialized " << std::endl;
   std::cout << " ATGAGG Pad Coordinates container initialized " << std::endl;
   fNumberPads = 40;
}

AtGAGGMap::~AtGAGGMap() = default;

void AtGAGGMap::Dump() {}

void AtGAGGMap::GeneratePadPlane() {}

Int_t AtGAGGMap::fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort)
{
   return 0;
}

XYPoint AtGAGGMap::CalcPadCenter(Int_t PadRef)
{
   return {0, 0};
}

Int_t AtGAGGMap::InhibitStrips(TString stripsFilePath)
{
   return 0;
}

void AtGAGGMap::ParseAtTPCMap(TXMLNode *node)
{

   Int_t fDigitizerID = -1000;
   Int_t fChannelID = -1000;
   Int_t fGAGGID = -1000;
   Int_t fLayerID = -1000;

   for (; node; node = node->GetNextNode()) {
      if (node->GetNodeType() == TXMLNode::kXMLElementNode) { // Element Node
         if (strcmp(node->GetNodeName(), "DigiID") == 0)
            fDigitizerID = atoi(node->GetText());
         if (strcmp(node->GetNodeName(), "ChannelID") == 0)
            fChannelID = atoi(node->GetText());
         if (strcmp(node->GetNodeName(), "LayerID") == 0)
            fLayerID = atoi(node->GetText());
         if (strcmp(node->GetNodeName(), "GAGGID") == 0)
            fGAGGID = atoi(node->GetText());
      }
   }
   AtPadReference ref = {fDigitizerID, 0, 0, fChannelID};
   std::cout << "loading GAGGMap XML: " << fDigitizerID << "." << 0 << "." << 0 << "." << fChannelID << ": "
             << "GAGGID = " << fGAGGID << ", LayerID = " << fLayerID << std::endl;
   fPadMap.insert(std::pair<AtPadReference, int>(ref, fGAGGID));
   fLayerMap.insert(std::pair<AtPadReference, int>(ref, fLayerID));

   fPadMapInverse.insert(std::pair<int, AtPadReference>(fGAGGID, ref));
}
ClassImp(AtGAGGMap)
