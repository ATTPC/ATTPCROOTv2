/*********************************************************************
 *   ATSI Mapping Class	AtSiMap.cxx			             *
 *   Author: Y. Ayyad            				     *
 *   Log: 13-02-2015 17:16 JST					     *
 *								     *
 *********************************************************************/

#include "AtSiMap.h"

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

AtSiMap::AtSiMap() : AtMap()
{
   AtPadCoord.resize(boost::extents[10240][3][2]);
   std::fill(AtPadCoord.data(), AtPadCoord.data() + AtPadCoord.num_elements(), 0);
   std::cout << " ATSI Map initialized " << std::endl;
   std::cout << " ATSI Pad Coordinates container initialized " << std::endl;
   fNumberPads = 128 * 4;
}

AtSiMap::~AtSiMap() = default;

void AtSiMap::Dump() {}

void AtSiMap::GeneratePadPlane() {}

Int_t AtSiMap::fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort)
{
   return 0;
}

XYPoint AtSiMap::CalcPadCenter(Int_t PadRef)
{
   return {0, 0};
}

Int_t AtSiMap::InhibitStrips(TString stripsFilePath)
{
   return 0;
}

void AtSiMap::ParseAtTPCMap(TXMLNode *node)
{

   Int_t fCoboID = -1000;
   Int_t fAsadID = -1000;
   Int_t fAgetID = -1000;
   Int_t fChannelID = -1000;
   Int_t fPadID = -1000;
   Int_t fFaceID = -1000;

   for (; node; node = node->GetNextNode()) {
      if (node->GetNodeType() == TXMLNode::kXMLElementNode) { // Element Node
         if (strcmp(node->GetNodeName(), "CoboID") == 0)
            fCoboID = atoi(node->GetText());
         if (strcmp(node->GetNodeName(), "AsadID") == 0)
            fAsadID = atoi(node->GetText());
         if (strcmp(node->GetNodeName(), "AgetID") == 0)
            fAgetID = atoi(node->GetText());
         if (strcmp(node->GetNodeName(), "ChannelID") == 0)
            fChannelID = atoi(node->GetText());
         if (strcmp(node->GetNodeName(), "PadID") == 0)
            fPadID = atoi(node->GetText());
         if (strcmp(node->GetNodeName(), "Siliconface") == 0)
            fFaceID = atoi(node->GetText());
      }
   }
   if (fFaceID < 0 || fFaceID > 3) {
      LOG(error) << "Warning! Silicon face " << fFaceID << " outside expected values or not found!" << std::endl;
   }
   fPadID = fPadID + 128 * fFaceID;
   AtPadReference ref = {fCoboID, fAsadID, fAgetID, fChannelID};
   std::cout << "loading SiMap XML: " << fCoboID << "." << fAsadID << "." << fAgetID << "." << fChannelID << ": "
             << fPadID << std::endl;
   fPadMap.insert(std::pair<AtPadReference, int>(ref, fPadID));

   fPadMapInverse.insert(std::pair<int, AtPadReference>(fPadID, ref));
}
ClassImp(AtSiMap)
