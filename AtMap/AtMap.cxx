#include "AtMap.h"
#include "FairLogger.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"

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

Bool_t AtMap::ParseInhibitMap(TString inimap, TString lowgmap, TString xtalkmap)
{

   std::ifstream *fIni = new std::ifstream(inimap.Data());
   std::ifstream *fLowg = new std::ifstream(lowgmap.Data());
   std::ifstream *fXtalk = new std::ifstream(xtalkmap.Data());

   Int_t pad;
   Bool_t IsIniPar = kTRUE;
   Bool_t IsLowgPar = kTRUE;
   Bool_t IsXtalkPar = kTRUE;

   std::cout << cYELLOW << __func__ << " - Parsing map for inhibited pads " << cNORMAL << std::endl;

   if (fIni->fail()) {
      std::cout << cRED << __func__
                << " = Warning : No Inhibit Pad Map found! Please, check the path. Current :" << inimap.Data()
                << cNORMAL << std::endl;
      IsIniPar = kFALSE;
   }
   if (fLowg->fail()) {
      std::cout << cRED << __func__
                << " = Warning : No Inhibit Pad Map found! Please, check the path. Current :" << lowgmap.Data()
                << cNORMAL << std::endl;
      IsLowgPar = kFALSE;
   }
   if (fXtalk->fail()) {
      std::cout << cRED << __func__
                << " = Warning : No Inhibit Pad Map found! Please, check the path. Current :" << xtalkmap.Data()
                << cNORMAL << std::endl;
      IsXtalkPar = kFALSE;
   }

   while (!fIni->eof() && IsIniPar) {
      *fIni >> pad;
      fIniPads.insert(pad);
   }

   while (!fLowg->eof() && IsLowgPar) {
      *fLowg >> pad;
      fIniPads.insert(pad);
   }

   while (!fXtalk->eof() && IsXtalkPar) {
      *fXtalk >> pad;
      fIniPads.insert(pad);
   }

   std::cout << cYELLOW << __func__ << "     " << fIniPads.size() << " pads added to inhibition list" << cNORMAL
             << std::endl;

   delete fIni;
   delete fLowg;
   delete fXtalk;

   return kTRUE;
}

Bool_t AtMap::GetIsInhibited(Int_t PadNum)
{
   // std::find(fIniPads.begin(),fIniPads.end(),PadNum);
   const Bool_t kIsInhibit = fIniPads.find(PadNum) != fIniPads.end();
   return kIsInhibit;
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

   /*std::vector<int> test;
     test.resize(4);
     test[0] = 0;
     test[1] = 0;
     test[2] = 0;
     test[3] = 0;*/

   for (; node; node = node->GetNextNode()) {
      if (node->GetNodeType() == TXMLNode::kXMLElementNode) { // Element node
         if (strcmp(node->GetNodeName(), "e17504_fission") == 0 || strcmp(node->GetNodeName(), "Lookup20150611") == 0 ||
             strcmp(node->GetNodeName(), "e18505") == 0 || strcmp(node->GetNodeName(), "LookupProto20150331") == 0 ||
             strcmp(node->GetNodeName(), "LookupProto10Be") == 0 ||
             strcmp(node->GetNodeName(), "LookupProto20181201v2") == 0 ||
             strcmp(node->GetNodeName(), "e12014_pad_mapping") == 0 ||
             strcmp(node->GetNodeName(), "e12014_pad_map_size") == 0 ||
             strcmp(node->GetNodeName(), "LookupProtoND") == 0) { // TODO Implement this as function parameter
            // cout<<node->GetNodeName()<<endl;
            // if(strcmp(node->GetNodeName(),"Lookup20141208") == 0){
            ParseAtTPCMap(node->GetChildren());
         } else
            std::cout << " AtTpcMap::ParseMapList - Node not found! Check node name" << std::endl;
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

   // Option 1: Int key - vector<int> value
   /*	std::map<int,std::vector<int>>::iterator it;
    std::ostream_iterator<int> ii (std::cout,", ");

    for(it=this->AtTPCPadMap.begin(); it!=this->AtTPCPadMap.end(); ++it){
    std::cout<<" [ "<<(*it).first<<", ";
    std::copy ((*it).second.begin(), (*it).second.end(), ii );
    std::cout<<"]"<<std::endl;;

    }

    std::map<int, std::vector<int>>::const_iterator ite = AtTPCPadMap.find(1);
    std::string value = it->second;
   */

   // Option 2: vector<int> key - int value

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

/*inline std::size_t std::hash<PadReference>::operator()(const PadReference &x) const
{
   auto wcobo = uint32_t(x.cobo);
   auto wasad = uint32_t(x.asad);
   auto waget = uint32_t(x.aget);
   auto wch = uint32_t(x.ch);


   }*/

ClassImp(AtMap)
