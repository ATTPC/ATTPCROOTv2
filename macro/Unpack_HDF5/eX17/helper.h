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

#ifndef __CLING__
#include "../build/include/AtAuxPad.h"
#include "../build/include/AtEvent.h"
#include "../build/include/AtPad.h"
#include "../build/include/AtPatternEvent.h"
#include "../build/include/AtRawEvent.h"
#include "../build/include/AtTpcMap.h"
#endif

// "public functions"
void loadRun(TString filePath, TString rawEventBranchName = "AtRawEvent",
             TString rawEventFilteredBranchName = "AtRawEventFiltered", TString eventBranchName = "AtEventH",
             TString patternBranchName = "AtPatternEvent");
bool loadEvent(ULong64_t eventNumber);
bool loadPad(int padNum);
bool nextEvent();

/**** "public" variables ******/
AtRawEvent *rawEventPtr;
AtRawEvent *rawEventFilteredPtr;
AtEvent *eventPtr;
AtPatternEvent *patternEventPtr;
AtTpcMap *tpcMap = nullptr;

/***** "Private" variables *******/
TChain *tpcTree = nullptr;
TTreeReader *reader = nullptr;
TTreeReaderValue<TClonesArray> *rawEventReader = nullptr;
TTreeReaderValue<TClonesArray> *eventFilteredReader = nullptr;
TTreeReaderValue<TClonesArray> *eventReader = nullptr;
TTreeReaderValue<TClonesArray> *patternEventReader = nullptr;

TH1F *hTrace = new TH1F("trace", "Trace", 512, 0, 511);

void loadRun(TString filePath, TString rawEventBranchName, TString rawEventFilteredBranchName, TString eventBranchName,
             TString patternEventBranchName)
{
   if (tpcMap == nullptr) {
      tpcMap = new AtTpcMap();
      tpcMap->ParseXMLMap(TString(gSystem->Getenv("VMCWORKDIR")) + "/scripts/e12014_pad_map_size.xml");
      tpcMap->AddAuxPad({10, 0, 0, 0}, "MCP_US");
      tpcMap->AddAuxPad({10, 0, 0, 34}, "TPC_Mesh");
      tpcMap->AddAuxPad({10, 0, 1, 0}, "MCP_DS");
      tpcMap->AddAuxPad({10, 0, 2, 34}, "IC");
      tpcMap->GeneratePadPlane();
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

   if (eventFilteredReader != nullptr)
      delete eventFilteredReader;
   if (tpcTree->GetBranch(rawEventFilteredBranchName) != nullptr)
      eventFilteredReader = new TTreeReaderValue<TClonesArray>(*reader, rawEventFilteredBranchName);
   else
      LOG(error) << "Could not find filtered raw event branch " << rawEventFilteredBranchName;

   if (patternEventReader != nullptr)
      delete patternEventReader;
   if (tpcTree->GetBranch(patternEventBranchName) != nullptr)
      patternEventReader = new TTreeReaderValue<TClonesArray>(*reader, patternEventBranchName);
   else
      LOG(error) << "Could not find PATTERNEVENT branch " << patternEventBranchName;
   // loadEvent(0);
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
   if (eventFilteredReader != nullptr)
      rawEventFilteredPtr = dynamic_cast<AtRawEvent *>((*eventFilteredReader)->At(0));
   if (patternEventReader != nullptr)
      patternEventPtr = dynamic_cast<AtPatternEvent *>((*patternEventReader)->At(0));

   return true;
}

bool loadPad(int padNum)
{
   auto pad = rawEventPtr->GetPad(padNum);
   if (pad == nullptr) {
      std::cout << "Pad number " << padNum << " is not valid for this event." << std::endl;
      return false;
   }

   for (int i = 0; i < 512; ++i)
      hTrace->SetBinContent(i + 1, pad->GetADC(i));
   return true;
}

bool nextEvent()
{
   auto ret = reader->Next();
   rawEventPtr = dynamic_cast<AtRawEvent *>((*rawEventReader)->At(0));
   eventPtr = dynamic_cast<AtEvent *>((*eventReader)->At(0));
   if (eventFilteredReader != nullptr)
      rawEventFilteredPtr = dynamic_cast<AtRawEvent *>((*eventFilteredReader)->At(0));
   if (patternEventReader != nullptr)
      patternEventPtr = dynamic_cast<AtPatternEvent *>((*patternEventReader)->At(0));

   return ret;
}
