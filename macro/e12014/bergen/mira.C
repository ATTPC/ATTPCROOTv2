#ifndef MIRA_C
#define MIRA_C
/* Macro for viewing the mesh, and pad signals of the tpc
 * for a run. Has two TCanvas, one for mesh the other for
 * a pad specified with loadPad.
 *
 * Usage: load with root -l  mira.C
 * Open a run with loadRun(pathToFile)
 * Load an event with loadEvent(eventNum)
 * Load a pad with  loadPad(int padNumber)
 */

// "public functions"
void loadRun(TString filePath);
void loadEvent(ULong64_t eventNumber);
TH1F *loadPad(int padNum);
TH1F *loadMesh();

TChain *tpcTree;
TTreeReader *reader;
TTreeReaderValue<TClonesArray> *rawEventReader;
TTreeReaderValue<TClonesArray> *eventReader;
AtRawEvent *rawEventPtr;
AtEvent *eventPtr;

// TCanvas *cMesh = new TCanvas("cMesh", "Mesh Trace", 600, 400);
TH1F *hMesh = new TH1F("mesh", "Mesh", 512, 0, 511);
// TCanvas *cPadTrace = new TCanvas("cPad", "Pad Trace", 600, 400);
TH1F *hTrace = new TH1F("trace", "Trace", 512, 0, 511);

void loadRun(TString filePath)
{
   tpcTree = new TChain("cbmsim");
   tpcTree->Add(filePath);
   reader = new TTreeReader(tpcTree);
   rawEventReader = new TTreeReaderValue<TClonesArray>(*reader, "AtRawEventFiltered");
   eventReader = new TTreeReaderValue<TClonesArray>(*reader, "AtEventFiltered");
}

void loadEvent(ULong64_t eventNumber)
{
   auto eventStatus = reader->SetEntry(eventNumber);
   if (eventStatus != TTreeReader::EEntryStatus::kEntryValid) {
      std::cout << "Failed to load entry: " << eventNumber << " with status " << eventStatus << std::endl;
      return;
   }

   rawEventPtr = dynamic_cast<AtRawEvent *>((*rawEventReader)->At(0));
   eventPtr = dynamic_cast<AtEvent *>((*eventReader)->At(0));
}

TH1F *loadMesh()
{
   for (int i = 0; i < 512; ++i)
      hMesh->SetBinContent(i + 1, eventPtr->GetMesh()[i]);
   return hMesh;
}

TH1F *loadPad(int padNum)
{
   auto pad = rawEventPtr->GetPad(padNum);
   if (pad == nullptr) {
      std::cout << "Pad number " << padNum << " is not valid for this event." << std::endl;
      return nullptr;
   }

   for (int i = 0; i < 512; ++i)
      hTrace->SetBinContent(i + 1, pad->GetADC(i));
   return hTrace;
}

#endif // ifndef MIRA_C
