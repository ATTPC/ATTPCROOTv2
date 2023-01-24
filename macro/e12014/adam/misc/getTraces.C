// Script to pull traces for a pad from a single event

void getTraces(int tpcRun = 195, int eventNum = 0, int padID = 3309)
{

   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpcRun));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");

   // Open file and skip header
   auto oFileName = "data/traces_" + std::to_string(tpcRun) + "," + std::to_string(padID) + ".dat";
   std::ofstream traceFile(oFileName);
   if (!traceFile.good())
      std::cout << "Failed to open file" << std::endl;

   // Create pad plane and load map
   TString mapFile = "e12014_pad_mapping.xml"; //"Lookup20150611.xml";
   // Set directories
   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + mapFile;

   auto fAtMapPtr = new AtTpcMap();
   fAtMapPtr->ParseXMLMap(mapDir.Data());
   fAtMapPtr->GeneratePadPlane();
   auto fPadPlane = fAtMapPtr->GetPadPlane();
   double fThreshold = 0;

   // Loop through every event
   reader.SetEntry(eventNum);

   // Get the event
   AtRawEvent *eventPtr = (AtRawEvent *)(event->At(0));
   auto pad = eventPtr->GetPad(padID);
   if (!pad) {
      std::cout << "Event had no pad " << padID << std::endl;
      std::cout << "Aborting!" << std::endl;
      return;
   }
   std::cout << "Getting trace for pad " << padID << " at " << fAtMapPtr->GetPadRef(padID) << endl;

   auto &rawTrace = pad->GetRawADC();
   auto &trace = pad->GetADC();

   // Write header
   traceFile << "TB,";
   traceFile << "Event" << eventNum << ",";
   traceFile << std::endl;

   // Loop through and print in csv file and fill pad plane
   for (int i = 0; i < 512; ++i) {

      traceFile << i;
      traceFile << "," << rawTrace[i];
      traceFile << "," << trace[i];
      traceFile << std::endl;
   }

   std::cout << "Done writing file" << std::endl;
}
