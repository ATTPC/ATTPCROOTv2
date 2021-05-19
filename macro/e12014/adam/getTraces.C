// Script to pull traces for a pad from a single event

void getTraces(int tpcRun = 200)
{

   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpcRun));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");

   // Open file and skip header
   std::ofstream traceFile("data/traces_" + std::to_string(tpcRun) + ".dat");
   if (!traceFile.good())
      std::cout << "Failed to open file" << std::endl;
   ;

   // Create pad plane and load map
   TString mapFile = "e12014_pad_mapping.xml"; //"Lookup20150611.xml";
   // Set directories
   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + mapFile;

   auto fAtMapPtr = new AtTpcMap();
   fAtMapPtr->ParseXMLMap(mapDir.Data());
   fAtMapPtr->GenerateAtTpc();
   auto fPadPlane = fAtMapPtr->GetAtTpcPlane();
   double fThreshold = 0;

   const int numEvents = 5;

   std::vector<std::vector<Short_t>> traces; // traces[event][time]

   // Loop through every event
   for (int i = 0; i < numEvents; ++i) {
      if (!reader.Next())
         break;

      //Get the event
      AtRawEvent *eventPtr = (AtRawEvent *)(event->At(0));
      auto numHits = eventPtr->GetNumPads();
      for (int i = 0; i < numHits; ++i) {
         // Get the padNum for the hit
         auto pad = eventPtr->GetPad(i);
         auto padNum = pad->GetPadNum();

         // Save the pad if we should
         if (padNum == padID) {
            std::vector<Short_t> trace;
            for (int t = 0; t < 512; ++t)
               trace.push_back(pad->GetRawADC(t));
            traces.push_back(trace);
         }
      }
   }

   // Write header
   traceFile << "TB,";
   for (int i = 0; i < numEvents; ++i)
      traceFile << "Event" << i << ",";
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
