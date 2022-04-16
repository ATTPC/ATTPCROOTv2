/* Helper functions for loading runs and pads
 *
 * Usage: load with root -l  helper.h
 * or #include helper.h
 * Open a run with loadRun(pathToFile)
 * Load an event with loadEvent(eventNum)
 * Load a pad with loadPad(int padNumber)
 *  - This fills the historgram hTrace
 */

#include <TChain.h>
#include <TH1.h>
#include <TString.h>
#include <TSystem.h>
#include <TTreeReader.h>

#include "../../../build/include/AtDecoder/AtAuxPad.h"
#include "../../../build/include/AtDecoder/AtEvent.h"
#include "../../../build/include/AtDecoder/AtPad.h"
#include "../../../build/include/AtDecoder/AtRawEvent.h"
#include "../../../build/include/AtRansac/AtRansac.h"
#include "../../../build/include/AtTpcMap.h"

// "public functions"
void loadRun(TString filePath, TString rawEventBranchName = "AtRawEventFiltered",
             TString eventBranchName = "AtEventFiltered", TString ransacBranchName = "AtRansac");
bool loadEvent(ULong64_t eventNumber);
void loadPad(int padNum);
bool nextEvent();

/**** "public" variables ******/
AtRawEvent *rawEventPtr;
AtEvent *eventPtr;
AtRANSACN::AtRansac *ransacPtr;
AtTpcMap *tpcMap = nullptr;

/***** "Private" variables *******/
TChain *tpcTree = nullptr;
TTreeReader *reader = nullptr;
TTreeReaderValue<TClonesArray> *rawEventReader = nullptr;
TTreeReaderValue<TClonesArray> *eventReader = nullptr;
TTreeReaderValue<TClonesArray> *ransacReader = nullptr;

TH1F *hTrace = new TH1F("trace", "Trace", 512, 0, 511);

void loadRun(TString filePath, TString rawEventBranchName, TString eventBranchName, TString ransacBranchName)
{
   if (tpcMap == nullptr) {
      tpcMap = new AtTpcMap();
      tpcMap->ParseXMLMap(TString(gSystem->Getenv("VMCWORKDIR")) + "/scripts/e12014_pad_map_size.xml");
      tpcMap->AddAuxPad({10, 0, 0, 0}, "MCP_US");
      tpcMap->AddAuxPad({10, 0, 0, 34}, "TPC_Mesh");
      tpcMap->AddAuxPad({10, 0, 1, 0}, "MCP_DS");
      tpcMap->AddAuxPad({10, 0, 2, 34}, "IC");
      tpcMap->GenerateAtTpc();
   }

   if (tpcTree != nullptr)
      delete tpcTree;
   tpcTree = new TChain("cbmsim");
   tpcTree->Add(filePath);

   if (reader != nullptr)
      delete reader;
   reader = new TTreeReader(tpcTree);

   if (rawEventReader != nullptr)
      delete rawEventReader;
   rawEventReader = new TTreeReaderValue<TClonesArray>(*reader, rawEventBranchName);

   if (eventReader != nullptr)
      delete eventReader;
   eventReader = new TTreeReaderValue<TClonesArray>(*reader, eventBranchName);

   if (ransacReader != nullptr)
      delete ransacReader;
   if (tpcTree->GetBranch(ransacBranchName) != nullptr)
      ransacReader = new TTreeReaderValue<TClonesArray>(*reader, ransacBranchName);
   else
      LOG(error) << "Could not find RANSAC branch " << ransacBranchName;
}

bool loadEvent(ULong64_t eventNumber)
{
   auto eventStatus = reader->SetEntry(eventNumber);
   if (eventStatus != TTreeReader::EEntryStatus::kEntryValid) {
      std::cout << "Failed to load entry: " << eventNumber << " with status " << eventStatus << std::endl;
      return false;
   }

   rawEventPtr = dynamic_cast<AtRawEvent *>((*rawEventReader)->At(0));
   eventPtr = dynamic_cast<AtEvent *>((*eventReader)->At(0));
   if (ransacReader != nullptr)
      ransacPtr = dynamic_cast<AtRANSACN::AtRansac *>((*ransacReader)->At(0));

   return true;
}

void loadPad(int padNum)
{
   auto pad = rawEventPtr->GetPad(padNum);
   if (pad == nullptr) {
      std::cout << "Pad number " << padNum << " is not valid for this event." << std::endl;
      return;
   }

   for (int i = 0; i < 512; ++i)
      hTrace->SetBinContent(i + 1, pad->GetADC(i));
}

bool nextEvent()
{
   auto ret = reader->Next();
   rawEventPtr = dynamic_cast<AtRawEvent *>((*rawEventReader)->At(0));
   eventPtr = dynamic_cast<AtEvent *>((*eventReader)->At(0));
   if (ransacReader != nullptr)
      ransacPtr = dynamic_cast<AtRANSACN::AtRansac *>((*ransacReader)->At(0));

   return ret;
}
