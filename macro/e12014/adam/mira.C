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
void loadPad(int padNum);

// "private" functions
void setMesh();

TChain *tpcTree;
TTreeReader *reader;
TTreeReaderValue<TClonesArray> *rawEventReader;
TTreeReaderValue<TClonesArray> *eventReader;
AtRawEvent *rawEventPtr;
AtEvent *eventPtr;

TCanvas *cMesh = new TCanvas("cMesh", "Mesh Trace", 600, 400);
TH1F *hMesh = new TH1F("mesh", "Mesh", 512, 0, 511);
TCanvas *cPadTrace = new TCanvas("cPad", "Pad Trace", 600, 400);
TH1F *hTrace = new TH1F("trace", "Trace", 512, 0, 511);

void loadRun(TString filePath)
{
   if (tpcTree != nullptr)
      delete tpcTree;
   tpcTree = new TChain("cbmsim");
   tpcTree->Add(filePath);

   if (reader != nullptr)
      delete reader;
   reader = new TTreeReader(tpcTree);

   if (rawEventReader != nullptr)
      delete rawEventReader;
   rawEventReader = new TTreeReaderValue<TClonesArray>(*reader, "AtRawEventFiltered");

   if (eventReader != nullptr)
      delete eventReader;
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

   setMesh();
}

void setMesh()
{
   for (int i = 0; i < 512; ++i)
      hMesh->SetBinContent(i + 1, eventPtr->GetMesh()[i]);
   cMesh->cd();
   hMesh->Draw("hist");
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
   cPadTrace->cd();
   hTrace->Draw("hist");
}
