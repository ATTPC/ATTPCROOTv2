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

/**
 * Returns a vector of pads in the beam region, most of which will contain the window edge.
 * The beam region is defined as all pads within radius of the center of the pad plane.
 */
std::vector<AtPad *> GetLikelyWindow(double radius = 20);
/**
 * Returns a vector of pads that likely contain the pad plane. It does this by selecting each
 * pad associated with a hit that occurs within zMin from the Pad Plane location (z = 0 mm) outside
 * the beam region (defined by radius).
 */
std::vector<AtPad *> GetLikelyPadPlane(double zMin = 30, double radius = 20);

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

std::vector<AtPad *> GetLikelyWindow(double radius)
{
   // get all the hits.
   std::vector<AtPad *> ret;
   for (const auto &hit : eventPtr->GetHitArray()) {
      if (hit.GetPosition().Rho() >= radius)
         continue;

      std::cout << " Good pad " << hit.GetPadNum() << " at " << hit.GetPosition() << std::endl;
      ret.push_back(rawEventPtr->GetPad(hit.GetPadNum()));
   }
   return ret;
}

std::vector<AtPad *> GetLikelyPadPlane(double zMin, double radius)
{

   std::vector<AtPad *> ret;
   // get all the hits sorted by time
   auto hitArray = eventPtr->GetHitArray();
   std::sort(hitArray.begin(), hitArray.end(),
             [](const AtHit &a, const AtHit &b) { return a.GetPosition().Z() < b.GetPosition().Z(); });

   for (const auto &hit : hitArray) {
      if (hit.GetPosition().Rho() < radius)
         continue;
      if (hit.GetPosition().Z() > zMin)
         continue;

      std::cout << " Good pad " << hit.GetPadNum() << " at " << hit.GetPosition() << std::endl;
      ret.push_back(rawEventPtr->GetPad(hit.GetPadNum()));
   }
   return ret;
}
#endif // ifndef MIRA_C
