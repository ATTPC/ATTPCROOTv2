// Script to pull traces for a pad from a single event

void getTracesInEvent(int tpcRun = 210, int eventNumber = 1)
{
   int numPads = 10; // if 0 get all pads

   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpackedReducedFiltered/run_%04d.root", tpcRun));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> event(reader, "AtRawEventFiltered");

   // Open file and skip header
   std::ofstream traceFile("data/traces_" + std::to_string(tpcRun) + ".dat");
   if (!traceFile.good())
      std::cout << "Failed to open file" << std::endl;

   // Create pad plane and load map
   TString mapFile = "e12014_pad_mapping.xml";
   // Set directories
   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + mapFile;

   auto fAtMapPtr = new AtTpcMap();
   fAtMapPtr->ParseXMLMap(mapDir.Data());
   fAtMapPtr->GenerateAtTpc();
   auto fPadPlane = fAtMapPtr->GetAtTpcPlane();
   double fThreshold = 0;

   std::vector<std::vector<Short_t>> traces; // traces[pad][time]
   std::vector<int> padIDs;                  // pad ID associated with traces

   // Get the event
   std::cout << reader.SetEntry(eventNumber) << std::endl;

   AtRawEvent *eventPtr = (AtRawEvent *)(event->At(0));
   if (numPads == 0 || numPads > eventPtr->GetNumPads())
      numPads = eventPtr->GetNumPads();

   for (int i = 0; i < numPads; ++i) {

      auto pad = eventPtr->GetPad(i);
      if (pad->IsAux()) {
         i--;
         continue;
      }
      padIDs.push_back(pad->GetPadNum());

      std::vector<Short_t> trace;
      for (int t = 0; t < 512; ++t)
         trace.push_back(pad->GetRawADC(t));
      traces.push_back(trace);
   }

   // Write header
   traceFile << "TB,";
   for (int i = 0; i < numPads; ++i)
      traceFile << padIDs[i] << ",";
   traceFile << std::endl;

   // Loop through and print in csv file and fill pad plane
   for (int i = 0; i < 512; ++i) {

      traceFile << i;
      for (auto &event : traces)
         traceFile << "," << event[i];
      traceFile << std::endl;
   }

   std::cout << "Done writing file" << std::endl;
}
