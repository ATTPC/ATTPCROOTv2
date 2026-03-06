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

   TH1F *histTDCRef = new TH1F("histTDCRef", "histTDCRef", 3000, 0, 150000);

   TH1F *histRF0 = new TH1F("histRF0", "histRF0", 3000, 0, 150000);
   TH1F *histRF1 = new TH1F("histRF1", "histRF1", 3000, 0, 150000);
   TH1F *histRF2 = new TH1F("histRF2", "histRF2", 3000, 0, 150000);
   TH1F *histRF3 = new TH1F("histRF3", "histRF3", 3000, 0, 150000);

   TH1F *histTDCRefRF0 = new TH1F("histTDCRefRF0", "histTDCRefRF0", 3000, 0, 6000);
   TH1F *histTDCRefRF1 = new TH1F("histTDCRefRF1", "histTDCRefRF1", 4000, 0, 8000);
   TH1F *histTDCRefRF2 = new TH1F("histTDCRefRF2", "histTDCRefRF2", 5000, 0, 10000);
   TH1F *histTDCRefRF3 = new TH1F("histTDCRefRF3", "histTDCRefRF3", 6000, 0, 12000);

   TH2F *histBeamPID = new TH2F("histBeamPID", "histBeamPID", 3000, 0, 12000, 400, 0, 2000);

   TH1F *histDeltaTEN_ATTPC2 = new TH1F("histDeltaTEN_ATTPC2", "histDeltaTEN_ATTPC2", 10000, 18.4, 18.5);
   TH1F *histDeltaTEN_ENPrev = new TH1F("histDeltaTEN_ENPrev", "histDeltaTEN_ENPrev", 1000, -1, 2);
   TH1F *histDeltaTATTPC2_ATTPC2Prev = new TH1F("histDeltaTATTPC2_ATTPC2Prev", "histDeltaTATTPC2_ATTPC2Prev", 1000, -1, 2);
   TH1F *histDeltaTEN_EnPrev_ATTPC2_ATTPC2Prev = new TH1F("histDeltaTEN_EnPrev_ATTPC2_ATTPC2Prev", "histDeltaTEN_EnPrev_ATTPC2_ATTPC2Prev", 90, -10, 80);

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
      TTreeReaderValue<TClonesArray> ICEventArray(unpackReader, "AtICEvent");
      TTreeReaderValue<TClonesArray> EventArray(unpackReader, "AtEventH");

      // Loop over events.
      ULong64_t prevENTS{};
      ULong64_t prevATTPC2TS{};
      for (int i = 0; i < nUnpackEvents; i++) {
         unpackReader.Next();

         AtENCourseEvent *ENEvent = (AtENCourseEvent *)ENCourseEventArray->At(0);
         AtICEvent *ICEvent = (AtICEvent *)ICEventArray->At(0);
         AtEvent *event = (AtEvent *)EventArray->At(0);

         if (!ENEvent->IsGood()) continue;

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

         histTDCRef->Fill(ENEvent->GetTDCRef());

         histRF0->Fill(ENEvent->GetRFToF(0));
         histRF1->Fill(ENEvent->GetRFToF(1));
         histRF2->Fill(ENEvent->GetRFToF(2));
         histRF3->Fill(ENEvent->GetRFToF(3));

         histTDCRefRF0->Fill(ENEvent->GetTDCRefRFToF(0));
         histTDCRefRF1->Fill(ENEvent->GetTDCRefRFToF(1));
         histTDCRefRF2->Fill(ENEvent->GetTDCRefRFToF(2));
         histTDCRefRF3->Fill(ENEvent->GetTDCRefRFToF(3));

         histBeamPID->Fill(ENEvent->GetTDCRefRFToF(3), ICEvent->GetADC());

         histDeltaTEN_ATTPC2->Fill(ENEvent->GetTimestamp()/1E6 - event->GetTimestamp(1)/1E6);
         histDeltaTEN_ENPrev->Fill(ENEvent->GetTimestamp()/1E6 - prevENTS/1E6);
         histDeltaTATTPC2_ATTPC2Prev->Fill(event->GetTimestamp(1)/1E6 - prevATTPC2TS/1E6);
         histDeltaTEN_EnPrev_ATTPC2_ATTPC2Prev->Fill(ENEvent->GetTimestamp() - prevENTS - event->GetTimestamp(1) + prevATTPC2TS);

         prevENTS = ENEvent->GetTimestamp();
         prevATTPC2TS = event->GetTimestamp(1);
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

   TCanvas *c9 = new TCanvas();
   histTDCRef->Draw();
   histTDCRef->GetXaxis()->SetTitle("TDC_{ref} [a.u.]");

   TCanvas *c10 = new TCanvas();
   histRF0->Draw();
   histRF0->GetXaxis()->SetTitle("ToF_{RF}[0] [a.u.]");

   TCanvas *c11 = new TCanvas();
   histRF1->Draw();
   histRF1->GetXaxis()->SetTitle("ToF_{RF}[1] [a.u.]");

   TCanvas *c12 = new TCanvas();
   histRF2->Draw();
   histRF2->GetXaxis()->SetTitle("ToF_{RF}[2] [a.u.]");

   TCanvas *c13 = new TCanvas();
   histRF3->Draw();
   histRF3->GetXaxis()->SetTitle("ToF_{RF}[3] [a.u.]");

   TCanvas *c14 = new TCanvas();
   histTDCRefRF0->Draw();
   histTDCRefRF0->GetXaxis()->SetTitle("ToF_{RF}[0] - TDC_{ref} [a.u.]");

   TCanvas *c15 = new TCanvas();
   histTDCRefRF1->Draw();
   histTDCRefRF1->GetXaxis()->SetTitle("ToF_{RF}[1] - TDC_{ref} [a.u.]");

   TCanvas *c16 = new TCanvas();
   histTDCRefRF2->Draw();
   histTDCRefRF2->GetXaxis()->SetTitle("ToF_{RF}[2] - TDC_{ref} [a.u.]");

   TCanvas *c17 = new TCanvas();
   histTDCRefRF3->Draw();
   histTDCRefRF3->GetXaxis()->SetTitle("ToF_{RF}[3] - TDC_{ref} [a.u.]");

   TCanvas *c18 = new TCanvas();
   histBeamPID->Draw("zcol");
   histBeamPID->GetXaxis()->SetTitle("ToF_{RF}[3] - TDC_{ref} [a.u.]");
   histBeamPID->GetYaxis()->SetTitle("ADC_{IC} [a.u.]");

   TCanvas *c19 = new TCanvas();
   histDeltaTEN_ATTPC2->Draw();
   histDeltaTEN_ATTPC2->GetXaxis()->SetTitle("t_{EN} - t_{ATTPC2} [s]");

   TCanvas *c20 = new TCanvas();
   histDeltaTEN_ENPrev->Draw();
   histDeltaTEN_ENPrev->GetXaxis()->SetTitle("t_{EN} - t_{ENPrev} [s]");

   TCanvas *c21 = new TCanvas();
   histDeltaTATTPC2_ATTPC2Prev->Draw();
   histDeltaTATTPC2_ATTPC2Prev->GetXaxis()->SetTitle("t_{ATTPC2} - t_{ATTPC2Prev} [s]");

   TCanvas *c22 = new TCanvas();
   histDeltaTEN_EnPrev_ATTPC2_ATTPC2Prev->Draw();
   histDeltaTEN_EnPrev_ATTPC2_ATTPC2Prev->GetXaxis()->SetTitle("t_{EN} - t_{ENPrev} - (t_{ATTPC2} - t_{ATTPC2Prev}) [#mus]");
}
