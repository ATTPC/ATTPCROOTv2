// Macro for testing the merging of attpc files and nscl files
// Outputs a graph of the difference in timestamps between the two events
// Saves a text file to "/mnt/analysis/e12014/HiRAEVT/merged/run-%04d.dat"
// With the what entry in the E12014 tree conrresponds to what fEventID in
// the TPC data.

// For testing NSCL:79 is longer and uncleared compared with TPC:309
// For testing NSCL:77 is longer and uncleared compared with TPC:308
void testMerge(int nsclRunNum, int tpcRunNum)
{

   TString nsclFile = TString::Format("/mnt/analysis/e12014/HiRAEVT/unpacked/run-%04d-00.root", nsclRunNum);
   TString attpcFile = TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpcRunNum);
   TString mergeFile = TString::Format("/mnt/analysis/e12014/HiRAEVT/merged/run-%04d.dat", nsclRunNum);

   // Open the data files
   TFile *fNSCL = new TFile(nsclFile, "READ");
   TFile *fTPC = new TFile(attpcFile, "READ");

   // Open the output file
   TFile *mergerFile = new TFile(mergeFile, "RECREATE");

   auto tN = (TTree *)fNSCL->Get("E12014");
   auto tT = (TTree *)fTPC->Get("cbmsim");

   mergerFile->cd();
   auto tMerge = new TTree();

   // Get the number of entries in both
   auto nEventsTPC = tT->GetEntries();
   auto nEventsNSCL = tN->GetEntries();
   auto nEvents = TMath::Min(nEventsTPC, nEventsNSCL);

   if (nEventsTPC != nEventsNSCL) {
      std::cout << "The number of events in NSCL and TPC don't match!" << std::endl;
      std::cout << "NSCL: " << nEventsNSCL << " events." << std::endl;
      std::cout << "TPC:  " << nEventsTPC << " events." << std::endl;
      std::cout << "Trying to match using the first: " << nEvents << " of each." << std::endl;
   }

   double x[nEvents - 1];
   double y[nEvents - 1];

   // Make the tree reader
   TTreeReader tpcReader("cbmsim", fTPC);
   TTreeReaderValue<TClonesArray> tpcEventArray(tpcReader, "ATRawEvent");
   TTreeReader nsclReader("E12014", fNSCL);
   TTreeReaderArray<ULong64_t> nsclTS(nsclReader, "tstamp");

   // nsclReader.Next();
   // tpcReader.Next();

   // auto tpcTS = ((ATRawEvent*) tpcEventArray->At(0))->GetTimestamp();
   // std::cout << tpcTimestamp << " " << nsclTS[1] << std::endl;

   //  auto offset = tpcTimestamp < nsclTS[1] ?
   //			       nsclTS[1] - tpcTimestamp
   //			       : tpcTimestamp - nsclTS[1];
   /*
     double offset = 0;

     if (nsclTS[1] > tpcTS)
       offset = (double)(nsclTS[1] - tpcTS);
     else
       offset = -(double)(tpcTS - nsclTS[1]);


     std::cout << "Timestamp offset is :" << offset << std::endl;
     x[0] = 0;
     y[0] = (double)nsclTS[1] - tpcTS;

     */
   nsclReader.Next();
   tpcReader.Next();

   std::cout << "#nsclEntry\tfEventID\tnsclTS\ttpcTimestamp" << std::endl;
   for (auto i = 0; i < nEvents - 1; ++i) {
      nsclReader.Next();
      tpcReader.Next();

      ATRawEvent *tpcEvent = (ATRawEvent *)tpcEventArray->At(0);

      x[i] = i;
      y[i] = (double)nsclTS[1] - tpcEvent->GetTimestamp();

      // std::cout << i << "," << tpcEvent->GetEventID() << "," << nsclTS[1] << "," << tpcEvent->GetTimestamp() << ","
      // << y[i] << std::endl;
      if (i == nEvents / 100)
         std::cout << "Event: " << i << "/" << nEvents << std::endl;

   } // End loop over events

   std::cout << "End loop" << std::endl;
   TGraph *gr = new TGraph(nEvents, x, y);
   gr->Draw();
}
