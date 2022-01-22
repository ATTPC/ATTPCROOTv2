// Script to pull traces for a pad from a single event

int getICTrace(int tpcRun = 210, int eventNumber = 1)
{

   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpackedReduced/run_%04d.root", tpcRun));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");

   // Open file and skip header
   std::ofstream traceFile(TString::Format("waveform_%d.dat", eventNumber).Data());
   if (!traceFile.good())
      std::cout << "Failed to open file" << std::endl;

   // Loop through every event
   if (reader.SetEntry(eventNumber) != TTreeReader::EEntryStatus::kEntryValid)
      return -1;

   // Get the event
   AtRawEvent *eventPtr = (AtRawEvent *)(event->At(0));
   auto numHits = eventPtr->GetNumPads();
   for (int i = 0; i < numHits; ++i) {
      auto pad = eventPtr->GetPad(i);
      if (!pad->IsAux())
         continue;
      if (pad->GetAuxName() != "IC")
         continue;

      std::cout << "Found IC" << std::endl;
      for (int t = 0; t < 512; ++t)
         traceFile << pad->GetRawADC(t) << std::endl;
      break;
   }

   std::cout << "Done writing file" << std::endl;
   return 0;
}
