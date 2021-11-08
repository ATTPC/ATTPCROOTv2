/* Helper functions for loading runs and pads
 *
 * Usage: load with root -l  helper.h
 * or #include helper.h
 * Open a run with loadRun(pathToFile)
 * Load an event with loadEvent(eventNum)
 * Load a pad with loadPad(int padNumber)
 *  - This fills the historgram hTrace
 */

// "public functions"
void loadRun(TString filePath, TString rawEventBranchName = "AtRawEventFiltered",
             TString eventBranchName = "AtEventFiltered");
bool loadEvent(ULong64_t eventNumber);
void loadPad(int padNum);

TChain *tpcTree;
TTreeReader *reader;
TTreeReaderValue<TClonesArray> *rawEventReader;
TTreeReaderValue<TClonesArray> *eventReader;
AtRawEvent *rawEventPtr;
AtEvent *eventPtr;

TH1F *hTrace = new TH1F("trace", "Trace", 512, 0, 511);

void loadRun(TString filePath, TString rawEventBranchName, TString eventBranchName)
{
   tpcTree = new TChain("cbmsim");
   tpcTree->Add(filePath);
   reader = new TTreeReader(tpcTree);
   rawEventReader = new TTreeReaderValue<TClonesArray>(*reader, rawEventBranchName);
   eventReader = new TTreeReaderValue<TClonesArray>(*reader, eventBranchName);
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
