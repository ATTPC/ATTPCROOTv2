#include "AtMap.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Rtypes.h>
#include <TCanvas.h>
#include <TDOMParser.h>
#include <TH2Poly.h>
#include <TObject.h> // for TObject
#include <TStyle.h>
#include <TXMLDocument.h>
#include <TXMLNode.h>

#include <boost/multi_array/base.hpp>
#include <boost/multi_array/extent_gen.hpp>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <memory>
#include <utility>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

using InhibitType = AtMap::InhibitType;
std::ostream &operator<<(std::ostream &os, const AtMap::InhibitType &t)
{
   switch (t) {
   case InhibitType::kNone: os << "kNone"; break;
   case InhibitType::kTotal: os << "kTotal"; break;
   case InhibitType::kLowGain: os << "kLowGain"; break;
   case InhibitType::kXTalk: os << "kXTalk"; break;
   case InhibitType::kBadPad: os << "kBadPad"; break;
   }
   return os;
}

AtMap::AtMap() : AtPadCoord(boost::extents[10240][3][2]), fPadPlane(nullptr)
{
   fCalibrationFunction = [](double *x, double *params) -> double {
      LOG(error)
         << "The calibration function was not set! Returning -999 by default. Please set the calibration function!";
      return -999;
   }; // Setting a default calibration function.
}

AtPadReference AtMap::GetNearestFPN(int padNum) const
{
   return GetNearestFPN(GetPadRef(padNum));
}

AtPadReference AtMap::GetNearestFPN(const AtPadReference &ref) const
{
   auto fpn = ref;
   if (ref.ch < 17)
      fpn.ch = 11;
   else if (ref.ch < 34)
      fpn.ch = 22;
   else if (ref.ch < 51)
      fpn.ch = 45;
   else
      fpn.ch = 56;

   return fpn;
}

bool AtMap::IsFPNchannel(const AtPadReference &ref) const
{
   return ref.ch == 11 || ref.ch == 22 || ref.ch == 45 || ref.ch == 56;
}

TH2Poly *AtMap::GetPadPlane()
{
   if (fPadPlane == nullptr)
      GeneratePadPlane();

   return dynamic_cast<TH2Poly *>(fPadPlane->Clone());
}

Int_t AtMap::GetPadNum(ROOT::Math::XYPoint point)
{
   if (fPadPlane == nullptr)
      GeneratePadPlane();
   auto binNum = fPadPlane->FindBin(point.X(), point.Y());
   if (binNum < 0)
      return -1;
   else
      return BinToPad(binNum);
}

Int_t AtMap::GetPadNum(const AtPadReference &PadRef) const
{

   // Option 1: Int key - vector<int> value
   // std::map<int, std::vector<int>>::const_iterator ite = fPadMap.find(1);
   // std::string value = it->second;
   // Option 2: vector<int> key - int value

   auto its = fPadMap.find(PadRef);

   // std::cout<<int(fPadMap.find(test) == fPadMap.end())<<endl;
   if (its == fPadMap.end()) {
      if (kDebug)
         std::cerr << " AtTpcMap::GetPadNum - Pad key not found - CoboID : " << PadRef.cobo
                   << "  AsadID : " << PadRef.asad << "  AgetID : " << PadRef.aget << "  ChannelID : " << PadRef.ch
                   << std::endl;
      return -1;
   }

   return (*its).second;
}

Bool_t AtMap::ParseInhibitMap(TString inimap, AtMap::InhibitType type)
{
   std::ifstream fIni(inimap.Data());

   Int_t pad = 0;

   LOG(info) << cYELLOW << __func__ << " - Parsing map for inhibited pads of type " << type << cNORMAL;
   if (fIni.fail()) {
      LOG(error) << cRED << " = Warning : No Inhibit Pad Map found! Please, check the path. Current :" << inimap.Data()
                 << cNORMAL;
      return false;
   }

   if (fIniPads.size() != 0)
      LOG(info) << "Will overriding any existing inhibited pads with new type";

   while (!fIni.eof()) {
      fIni >> pad;
      InhibitPad(pad, type);
   }

   LOG(info) << cYELLOW << fIniPads.size() << " pads in inhibition list." << cNORMAL;
   return true;
}

void AtMap::InhibitPad(AtPadReference padRef, AtMap::InhibitType type)
{
   auto pad = fIniPads.find(padRef);
   if (pad == fIniPads.end() || pad->second < type)
      fIniPads[padRef] = type;
}

AtMap::InhibitType AtMap::IsInhibited(AtPadReference padRef)
{
   auto pad = fIniPads.find(padRef);
   if (pad == fIniPads.end())
      return InhibitType::kNone;
   else
      return pad->second;
}

int AtMap::GetPadSize(int padNum)
{
   if (fPadSizeMap.find(padNum) == fPadSizeMap.end())
      return -1000;
   return fPadSizeMap[padNum];
}
void AtMap::ParseAtTPCMap(TXMLNode *node)
{

   Int_t fCoboID = -1000;
   Int_t fAsadID = -1000;
   Int_t fAgetID = -1000;
   Int_t fChannelID = -1000;
   Int_t fPadID = -1000;
   Int_t fSizeID = -1000;

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
         if (strcmp(node->GetNodeName(), "SizeID") == 0)
            fSizeID = atoi(node->GetText());
      }
   }
   AtPadReference ref = {fCoboID, fAsadID, fAgetID, fChannelID};

   fPadMap.insert(std::pair<AtPadReference, int>(ref, fPadID));
   fPadMapInverse.insert(std::pair<int, AtPadReference>(fPadID, ref));
   fPadSizeMap.insert(std::pair<int, int>(fPadID, fSizeID));
}

void AtMap::ParseMapList(TXMLNode *node)
{

   for (; node; node = node->GetNextNode()) {
      if (node->GetNodeType() == TXMLNode::kXMLElementNode) { // Element node
         if (strcmp(node->GetNodeName(), "e17504_fission") == 0 || strcmp(node->GetNodeName(), "Lookup20150611") == 0 ||
             strcmp(node->GetNodeName(), "e18505") == 0 || strcmp(node->GetNodeName(), "LookupProto20150331") == 0 ||
             strcmp(node->GetNodeName(), "LookupProto10Be") == 0 || "ANL2023.xml" ||
             strcmp(node->GetNodeName(), "LookupProto20181201v2") == 0 ||
             strcmp(node->GetNodeName(), "LookupProtoX17") == 0 ||
             strcmp(node->GetNodeName(), "e12014_pad_mapping") == 0 ||
             strcmp(node->GetNodeName(), "e12014_pad_map_size") == 0 ||
             strcmp(node->GetNodeName(), "LookupGADGET08232021") == 0 ||
             strcmp(node->GetNodeName(), "Lookup20141208") == 0 ||
             strcmp(node->GetNodeName(), "LookupSpecMATnoScint") == 0 ||
             strcmp(node->GetNodeName(), "LookupSpecMATnoScintHisto") == 0 ||
             strcmp(node->GetNodeName(), "LookupSpecMATnoScint3seg") == 0 ||
             strcmp(node->GetNodeName(), "LookupProtoND") == 0) { // TODO Implement this as function parameter

            ParseAtTPCMap(node->GetChildren());
         } else
            std::cout << " AtTpcMap::ParseMapList - Node not found! Check node name" << std::endl;
         // std::cout <<node->GetNodeName()<<std::endl;
      }
   }
   kIsParsed = true;
}

Bool_t AtMap::ParseXMLMap(Char_t const *xmlfile)
{

   auto domParser = std::make_unique<TDOMParser>();
   domParser->SetValidate(false);
   Int_t parsecode = domParser->ParseFile(xmlfile);
   if (parsecode < 0) {
      std::cerr << domParser->GetParseCodeMessage(parsecode) << std::endl;
      return false;
   }
   TXMLNode *node = domParser->GetXMLDocument()->GetRootNode();
   ParseMapList(node->GetChildren());
   // itrEnd = pmap.end();

   LOG(info) << "Pad map has an average load of " << fPadMap.load_factor() << " and a max load of "
             << fPadMap.max_load_factor() << " with buckets " << fPadMap.bucket_count() << " for " << fPadMap.size()
             << " pads.";

   return true;
}

Bool_t AtMap::ParseCalibrationParameters(TString calibrationFilePath, int calibrationParameterNumber)
{
   fCalibrationParameterNumber = calibrationParameterNumber;
   LOG(warning)
      << "Trying to pass " << fCalibrationParameterNumber
      << " to the calibration parameter arrays. Please make sure the function you are setting as calibration requires "
      << fCalibrationParameterNumber << " parameters!";

   std::ifstream calibrationFile(calibrationFilePath.Data());
   std::string line;

   if (!calibrationFile.is_open()) {
      LOG(error) << "Failed to open file: " << calibrationFilePath.Data();
      return false;
   }

   // Skip header line.
   std::getline(calibrationFile, line);

   int lineNumber{1};
   while (std::getline(calibrationFile, line)) {
      lineNumber++;

      std::stringstream ss(line);
      std::string entry;

      // Variables where to read the lines.
      int padID{};
      std::vector<double> parameters;

      // First column should be the padID.
      std::getline(ss, entry, ',');
      padID = std::stoi(entry);

      while (std::getline(ss, entry, ',')) {
         if (parameters.size() == fCalibrationParameterNumber) {
            LOG(warning) << "Promised " << fCalibrationParameterNumber << " parameters but reached parameter "
                         << parameters.size() + 1 << " on line " << lineNumber
                         << "! Ignoring rest of parameters on the line!";
            break;
         }
         parameters.push_back(std::stod(entry));
      }

      if (parameters.size() < fCalibrationParameterNumber) {
         LOG(fatal) << "Promised " << fCalibrationParameterNumber << " parameters but got only " << parameters.size()
                    << " on line " << lineNumber << "! Parameter vector of pad " << padID << " will not be set!";
         std::vector<double> emptyVector;
         fCalibrationParametersMap.insert(std::pair<int, std::vector<double>>(padID, emptyVector));
         continue;
      }

      fCalibrationParametersMap.insert(std::pair<int, std::vector<double>>(padID, parameters));
   }

   return true;
}

double AtMap::GetCalibratedELoss(int padID, double ADC) const
{
   auto entry = fCalibrationParametersMap.find(padID);
   if (entry == fCalibrationParametersMap.end()) {
      LOG(warning) << "Pad " << padID
                   << " did not have an entry in the calibration parameters file... Returning ELoss = -999 by default.";
      return -999;
   }

   std::vector<double> parameters = entry->second;
   if (parameters.empty()) {
      LOG(warning) << "Pad " << padID
                   << " had an issue when reading the calibration parameters file... Returning ELoss = -999 by "
                      "default. Please check the file!";
      return -999;
   }

   return fCalibrationFunction(&ADC, &parameters[0]);
}

bool AtMap::IsCalibrationSet() const
{
   return !fCalibrationParametersMap.empty();
}

Bool_t AtMap::DumpAtTPCMap()
{
   if (!kIsParsed) {

      std::cout << " AtTpcMap::DumpAtTPCMap Error : Pad plane has not been generated or parsed - Exiting... "
                << std::endl;

      return false;
   }

   std::ostream_iterator<int> ii(std::cout, ", ");

   for (auto &it : this->fPadMap) {
      std::cout << " [ " << it.second << ", ";
      std::cout << it.first.cobo << "," << it.first.asad << "," << it.first.aget << "," << it.first.ch;
      std::cout << "]" << std::endl;
      ;
   }

   return true;
}

bool AtMap::AddAuxPad(const AtPadReference &ref, std::string auxName)
{
   auto emplacePair = fAuxPadMap.emplace(ref, auxName);
   std::cout << cGREEN << " Auxiliary channel added " << fAuxPadMap[ref] << " - Hash "
             << std::hash<AtPadReference>()(ref) << cNORMAL << "\n";

   return emplacePair.second;
}
bool AtMap::IsAuxPad(const AtPadReference &ref) const
{
   return fAuxPadMap.find(ref) != fAuxPadMap.end();
}
std::string AtMap::GetAuxName(const AtPadReference &ref) const
{
   if (IsAuxPad(ref))
      return fAuxPadMap.find(ref)->second;
   else
      return "";
}
AtPadReference AtMap::GetPadRef(int padNum) const
{
   if (fPadMapInverse.find(padNum) == fPadMapInverse.end())
      return {};
   return fPadMapInverse.at(padNum);
}

void AtMap::drawPadPlane()
{
   // NOLINTNEXTLINE (memory belongs t root)
   fPadPlaneCanvas = new TCanvas("padPlaneCanvas", "Pad Plane", 1000, 1000);
   gStyle->SetPalette(1);
   fPadPlane->Draw("col");
}

ClassImp(AtMap)
