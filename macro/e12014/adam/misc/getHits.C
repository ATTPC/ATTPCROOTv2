// Script to pull the hits from a single event and output them in a CSV file

void getHits(int tpcRun = 210, int eventNumber = 0)
{

   double fThreshold = 0; // Only save hits if they're above this threshold

   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpackedReducedFiltered/run_%04d.root", tpcRun));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> event(reader, "AtEventH");

   // Open file and skip header
   std::ofstream traceFile("data/hits_" + std::to_string(tpcRun) + "_" + std::to_string(eventNumber) + ".dat");
   if (!traceFile.good())
      std::cout << "Failed to open file" << std::endl;

   // Create pad plane and load map
   TString mapFile = "e12014_pad_mapping.xml"; //"Lookup20150611.xml";
   // Set directories
   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + mapFile;

   auto fAtMapPtr = new AtTpcMap();
   fAtMapPtr->ParseXMLMap(mapDir.Data());
   fAtMapPtr->GenerateAtTpc();
   auto fPadPlane = fAtMapPtr->GetAtTpcPlane();

   std::vector<std::vector<Short_t>> traces; // traces[event][time]

   // Get the event to read
   reader.SetEntry(eventNumber);
   // Get the event
   AtEvent *eventPtr = (AtEvent *)(event->At(0));
   auto numHits = eventPtr->GetNumHits();

   // Write header
   traceFile << "x,y,z,charge" << std::endl;

   for (int i = 0; i < numHits; ++i) {
      // Get the padNum for the hit
      auto hit = eventPtr->GetHit(i);
      auto position = hit->GetPosition();
      auto charge = hit->GetCharge();

      traceFile << position.X() << "," << position.Y() << "," << position.Z() << "," << charge << std::endl;
   }
   traceFile.close();
   std::cout << "Done writing file" << std::endl;
}
