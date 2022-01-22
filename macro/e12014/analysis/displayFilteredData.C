
void displayFilteredData(int tpcRun = 206)
{

   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/filterTesting/run_%04d.root", tpcRun));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");
   TTreeReaderValue<TClonesArray> eventFilter(reader, "AtRawEventFiltered");

   const int numEvents = 1;

   // Loop through every event
   for (int i = 0; i < numEvents; ++i) {
      if (!reader.Next())
         break;

      // Get the unfiltered and filtered events
      AtRawEvent *eventPtr = (AtRawEvent *)(event->At(0));
      AtRawEvent *filterPtr = (AtRawEvent *)(eventFilter->At(0));

      // Get the pad to display on both
      auto pad = eventPtr->GetPad(0);
      auto padNum = pad->GetPadNum();

      // Pointer to hold the filtered pad to display
      AtPad *filterPad;

      // Loop through and look for filtered pad.
      for (int i = 0; i < filterPtr->GetNumPads(); ++i) {
         filterPad = filterPtr->GetPad(i);
         if (filterPad->GetPadNum() == padNum)
            break;
      } // end loop over pads

      // Show the two pads
      TH1F *raw = new TH1F("rawHist", "RawData", 512, 0, 511);
      TH1F *filtered = new TH1F("filteredHist", "FilteredData", 512, 0, 511);
      for (int i = 0; i < 512; ++i) {
         raw->SetBinContent(i, pad->GetADC(i));
         filtered->SetBinContent(i, filterPad->GetADC(i));
      }
      TCanvas *c1 = new TCanvas();
      raw->Draw();
      TCanvas *c3 = new TCanvas();
      filtered->Draw();
   }
}
