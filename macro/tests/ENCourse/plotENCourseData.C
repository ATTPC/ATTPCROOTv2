void plotENCourseData()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run
   // Histogram definitions.
   TH2F *histF2EntrancePosition = new TH2F("histF2EntrancePosition", "histF2EntrancePosition", 2000, -1000, 1000, 2000, -1000, 1000);
   TH2F *histF2ExitPosition = new TH2F("histF2ExitPosition", "histF2ExitPosition", 2000, -1000, 1000, 2000, -1000, 1000);
   TH1F *histF2PolarAngle = new TH1F("histF2PolarAngle", "histF2PolarAngle", 180, 0, 180);
   TH1F *histF2AzimutalAngle = new TH1F("histF2AzimutalAngle", "histF2AzimutalAngle", 360, -180, 180);

   TH2F *histF3EntrancePosition = new TH2F("histF3EntrancePosition", "histF3EntrancePosition", 2000, -1000, 1000, 2000, -1000, 1000);
   TH2F *histF3ExitPosition = new TH2F("histF3ExitPosition", "histF3ExitPosition", 2000, -1000, 1000, 2000, -1000, 1000);
   TH1F *histF3PolarAngle = new TH1F("histF3PolarAngle", "histF3PolarAngle", 180, 0, 180);
   TH1F *histF3AzimutalAngle = new TH1F("histF3AzimutalAngle", "histF3AzimutalAngle", 360, -180, 180);

   // All runs.
   std::vector runNums = {5074};

   for (int runNum: runNums) {
      // Open the unpacked file and get the TTree.
      TString unpackFileName = TString::Format("/media/aurio/Cris/E510/UnpackerOutput/run_%04d_testENCourseMerger.root", runNum);
      TFile *unpackFile = new TFile(unpackFileName, "READ");
      TTree *unpackTree = (TTree *)unpackFile->Get("cbmsim");
      int nUnpackEvents = unpackTree->GetEntries();
      std::cout << " Number of unpacked events in run " << runNum << ": " << nUnpackEvents << std::endl;

      // Creare the TTreeReader to read the AtENCourseEvents.
      TTreeReader unpackReader("cbmsim", unpackFile);
      TTreeReaderValue<TClonesArray> ENCourseEventArray(unpackReader, "AtENCourseEvent");

      // Loop over events.
      for (int i = 0; i < nUnpackEvents; i++) {
         unpackReader.Next();

         AtENCourseEvent *ENEvent = (AtENCourseEvent *)ENCourseEventArray->At(0);

         auto F2PPACs = ENEvent->GetF2PPACs();
         auto F2EntrancePosition = F2PPACs.GetEntrancePosition();
         auto F2ExitPosition = F2PPACs.GetExitPosition();
         Double_t trackF2PolarAngle = F2PPACs.GetTrackPolarAngle() * TMath::RadToDeg();
         Double_t trackF2AzimutalAngle = F2PPACs.GetTrackAzimutalAngle() * TMath::RadToDeg();

         histF2EntrancePosition->Fill(F2EntrancePosition.X(), F2EntrancePosition.Y());
         histF2ExitPosition->Fill(F2ExitPosition.X(), F2ExitPosition.Y());
         histF2PolarAngle->Fill(trackF2PolarAngle);
         histF2AzimutalAngle->Fill(trackF2AzimutalAngle);

         auto F3PPACs = ENEvent->GetF3PPACs();
         auto F3EntrancePosition = F3PPACs.GetEntrancePosition();
         auto F3ExitPosition = F3PPACs.GetExitPosition();
         Double_t trackF3PolarAngle = F3PPACs.GetTrackPolarAngle() * TMath::RadToDeg();
         Double_t trackF3AzimutalAngle = F3PPACs.GetTrackAzimutalAngle() * TMath::RadToDeg();

         histF3EntrancePosition->Fill(F3EntrancePosition.X(), F3EntrancePosition.Y());
         histF3ExitPosition->Fill(F3ExitPosition.X(), F3ExitPosition.Y());
         histF3PolarAngle->Fill(trackF3PolarAngle);
         histF3AzimutalAngle->Fill(trackF3AzimutalAngle);
      }
      unpackFile->Close();
   }

   // Draw histograms in TCanvas.
   TCanvas *c = new TCanvas();
   histF2EntrancePosition->Draw("zcol");
   histF2EntrancePosition->GetXaxis()->SetTitle("x [mm]");
   histF2EntrancePosition->GetYaxis()->SetTitle("y [mm]");

   TCanvas *c2 = new TCanvas();
   histF2ExitPosition->Draw("zcol");
   histF2ExitPosition->GetXaxis()->SetTitle("x [mm]");
   histF2ExitPosition->GetYaxis()->SetTitle("y [mm]");

   TCanvas *c3 = new TCanvas();
   histF2PolarAngle->Draw();
   histF2PolarAngle->GetXaxis()->SetTitle("#theta [deg]");

   TCanvas *c4 = new TCanvas();
   histF2AzimutalAngle->Draw();
   histF2AzimutalAngle->GetXaxis()->SetTitle("#phi [deg]");

   TCanvas *c5 = new TCanvas();
   histF3EntrancePosition->Draw("zcol");
   histF3EntrancePosition->GetXaxis()->SetTitle("x [mm]");
   histF3EntrancePosition->GetYaxis()->SetTitle("y [mm]");

   TCanvas *c6 = new TCanvas();
   histF3ExitPosition->Draw("zcol");
   histF3ExitPosition->GetXaxis()->SetTitle("x [mm]");
   histF3ExitPosition->GetYaxis()->SetTitle("y [mm]");

   TCanvas *c7 = new TCanvas();
   histF3PolarAngle->Draw();
   histF3PolarAngle->GetXaxis()->SetTitle("#theta [deg]");

   TCanvas *c8 = new TCanvas();
   histF3AzimutalAngle->Draw();
   histF3AzimutalAngle->GetXaxis()->SetTitle("#phi [deg]");
}
