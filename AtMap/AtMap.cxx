#include "AtMap.h"

#include "FairLogger.h"

#include "TH2Poly.h"
#include "TDOMParser.h"
#include "TXMLNode.h"

#include <iostream>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

std::ostream &operator<<(std::ostream &os, const AtMap::InhibitType &t)
{
   switch (t) {
   case AtMap::kNone: os << "kNone"; break;
   case AtMap::kTotal: os << "kTotal"; break;
   case AtMap::kLowGain: os << "kLowGain"; break;
   case AtMap::kXTalk: os << "kXTalk"; break;
   }
   return os;
}

AtMap::AtMap() : AtPadCoord(boost::extents[10240][3][2]) {}

AtMap::~AtMap() {}

Int_t AtMap::GetPadNum(const PadReference &PadRef) const
{

   // Option 1: Int key - vector<int> value
   // std::map<int, std::vector<int>>::const_iterator ite = AtTPCPadMap.find(1);
   // std::string value = it->second;
   // Option 2: vector<int> key - int value

   auto its = AtTPCPadMap.find(PadRef);

   // std::cout<<int(AtTPCPadMap.find(test) == AtTPCPadMap.end())<<endl;
   if (its == AtTPCPadMap.end()) {
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

   Int_t pad;

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
      inhibitPad(pad, type);
   }

   LOG(info) << cYELLOW << fIniPads.size() << " pads in inhibition list." << cNORMAL;
   return true;
}

void AtMap::inhibitPad(Int_t padNum, AtMap::InhibitType type)
{
   auto pad = fIniPads.find(padNum);
   if (pad == fIniPads.end() || pad->second < type)
      fIniPads[padNum] = type;
}
AtMap::InhibitType AtMap::IsInhibited(Int_t PadNum)
{
   auto pad = fIniPads.find(PadNum);
   if (pad == fIniPads.end())
      return kNone;
   else
      return pad->second;
}

int AtMap::GetPadSize(int padNum)
{
   if (AtTPCPadSize.find(padNum) == AtTPCPadSize.end())
      return -1000;
   return AtTPCPadSize[padNum];
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
   PadReference ref = {fCoboID, fAsadID, fAgetID, fChannelID};

   AtTPCPadMap.insert(std::pair<PadReference, int>(ref, fPadID));
   AtTPCPadMapInverse.insert(std::pair<int, PadReference>(fPadID, ref));
   AtTPCPadSize.insert(std::pair<int, int>(fPadID, fSizeID));
}

void AtMap::ParseMapList(TXMLNode *node)
{

   for (; node; node = node->GetNextNode()) {
      if (node->GetNodeType() == TXMLNode::kXMLElementNode) { // Element node
         if (strcmp(node->GetNodeName(), "e17504_fission") == 0 || strcmp(node->GetNodeName(), "Lookup20150611") == 0 ||
             strcmp(node->GetNodeName(), "e18505") == 0 || strcmp(node->GetNodeName(), "LookupProto20150331") == 0 ||
             strcmp(node->GetNodeName(), "LookupProto10Be") == 0 ||
             strcmp(node->GetNodeName(), "LookupProto20181201v2") == 0 ||
             strcmp(node->GetNodeName(), "LookupProtoX17") == 0 ||
             strcmp(node->GetNodeName(), "e12014_pad_mapping") == 0 ||
             strcmp(node->GetNodeName(), "e12014_pad_map_size") == 0 ||
             strcmp(node->GetNodeName(), "LookupGADGET08232021") == 0 ||
             strcmp(node->GetNodeName(), "Lookup20141208") == 0 ||
             strcmp(node->GetNodeName(), "LookupSpecMATnoScint") == 0 ||
             strcmp(node->GetNodeName(), "LookupProtoND") == 0) { // TODO Implement this as function parameter

            ParseAtTPCMap(node->GetChildren());
         } else
            std::cout << " AtTpcMap::ParseMapList - Node not found! Check node name" << std::endl;
         // std::cout <<node->GetNodeName()<<std::endl;
      }
   }

   kIsParsed = 1;
}

Bool_t AtMap::ParseXMLMap(Char_t const *xmlfile)
{

   TDOMParser *domParser = new TDOMParser();
   domParser->SetValidate(false);
   Int_t parsecode = domParser->ParseFile(xmlfile);
   if (parsecode < 0) {
      std::cerr << domParser->GetParseCodeMessage(parsecode) << std::endl;
      return false;
   }
   TXMLNode *node = domParser->GetXMLDocument()->GetRootNode();
   ParseMapList(node->GetChildren());
   // itrEnd = pmap.end();
   delete domParser;

   LOG(INFO) << "Pad map has an average load of " << AtTPCPadMap.load_factor() << " and a max load of "
             << AtTPCPadMap.max_load_factor() << " with buckets " << AtTPCPadMap.bucket_count() << " for "
             << AtTPCPadMap.size() << " pads.";

   return true;
}

Bool_t AtMap::DumpAtTPCMap()
{
   if (!fPadInd || !kIsParsed) {

      std::cout << " AtTpcMap::DumpAtTPCMap Error : Pad plane has not been generated or parsed - Exiting... "
                << std::endl;

      return false;
   }

   std::ostream_iterator<int> ii(std::cout, ", ");

   for (auto it = this->AtTPCPadMap.begin(); it != this->AtTPCPadMap.end(); ++it) {
      std::cout << " [ " << (*it).second << ", ";
      std::cout << it->first.cobo << "," << it->first.asad << "," << it->first.aget << "," << it->first.ch;
      std::cout << "]" << std::endl;
      ;
   }

   return true;
}

bool AtMap::AddAuxPad(const PadReference &ref, std::string auxName)
{
   auto emplacePair = fAuxPadMap.emplace(ref, auxName);
   std::cout << cGREEN << " Auxiliary channel added " << fAuxPadMap[ref] << " - Hash " << std::hash<PadReference>()(ref)
             << cNORMAL << "\n";

   return emplacePair.second;
}
bool AtMap::IsAuxPad(const PadReference &ref) const
{
   return fAuxPadMap.find(ref) != fAuxPadMap.end();
}
std::string AtMap::GetAuxName(const PadReference &ref) const
{
   if (IsAuxPad(ref))
      return fAuxPadMap.find(ref)->second;
   else
      return "";
}
PadReference AtMap::GetPadRef(int padNum) const
{
   if (AtTPCPadMapInverse.find(padNum) == AtTPCPadMapInverse.end())
      return PadReference();
   return AtTPCPadMapInverse.at(padNum);
}

bool operator<(const PadReference &l, const PadReference &r)
{
   return std::hash<PadReference>()(l) < std::hash<PadReference>()(r);
}

bool operator==(const PadReference &l, const PadReference &r)
{
   return l.cobo == r.cobo && l.asad == r.asad && l.aget == r.aget && l.ch == r.ch;
}

ClassImp(AtMap)
