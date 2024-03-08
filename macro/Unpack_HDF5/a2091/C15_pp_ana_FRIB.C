Double_t GetNPeaksHRS(std::vector<Int_t> *timeMax, std::vector<Float_t> *adcMax, double *adc_test)
{
   TSpectrum *s = new TSpectrum();
   Double_t dest[2048];
   // Int_t nfound = s->Search(h1_test,2," ",0.25);//2 and 0.15
   Int_t nfound;
   nfound = s->SearchHighRes(adc_test, dest, 2048, 20, 20, kFALSE, 1, kFALSE, 1);
   // nfound = s->SearchHighRes(adc_test, dest, 512, 2, 2, kTRUE, 3, kTRUE, 3);

   for (auto iPeak = 0; iPeak < nfound; ++iPeak) {

      Int_t time = (Int_t)(ceil((s->GetPositionX())[iPeak]));
      timeMax->push_back(time);
      adcMax->push_back(adc_test[time]);
   }

   delete s;
   return nfound;
}

void C15_pp_ana_FRIB(TString fileName = "run_0138_FRIB")
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   std::vector<TString> files{fileName};
   TString filesuffix = ".root";

   TH1F *hwaveform = new TH1F("waveform", "waveform", 2048, 0, 2047);
   TH1F *hmultiplicity = new TH1F("multiplicity", "multiplicity", 10, 0, 10);
   TH1F *henergy = new TH1F("energy", "energy", 4096, 0, 4095);
   TH1F *htime = new TH1F("time", "time", 2048, 0, 2047);

   // output variables
   ULong64_t timestamp;
   Int_t mult;
   std::vector<Float_t> energy;
   std::vector<Float_t> time;
   std::string fileNames;
   std::string eventName;

   // Output file
   TString outputName = files.at(0) + "_sorted" + filesuffix;
   TFile *outfile = TFile::Open(outputName.Data(), "RECREATE");
   TTree *outtree = new TTree("FRIB_output_tree", "FRIB_output_tree");
   outtree->Branch("timestamp", &timestamp, "timestamp/l");
   outtree->Branch("fileNames", &fileNames, "fileNames/C");
   outtree->Branch("eventName", &eventName);
   outtree->Branch("mult", &mult, "mult/i");
   outtree->Branch("energy", &energy);
   outtree->Branch("time", &time);

   for (auto iFile : files) {

      TFile *file = new TFile((iFile + filesuffix).Data(), "READ");
      fileNames = (iFile + filesuffix).Data();
      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtRawEvent");

      for (Int_t i = 0; i < nEvents - 1; i++) { // NB to prevent crashing due to the last event being empty

         if (i % 1000 == 0)
            std::cout << " Event Number : " << i << "\n";

         Reader1.Next();
         energy.clear();
         time.clear();

         auto *rawEvent = (AtRawEvent *)eventArray->At(0);
         timestamp = rawEvent->GetTimestamp();
         eventName = rawEvent->GetEventName();
         std::vector<Float_t> ICVec;
         std::vector<Int_t> ICTimeVec;

         if (rawEvent) {

            auto genTraces = &rawEvent->GetGenTraces();

            if (auto trace = genTraces->at(0).get()) {
               auto adc = &trace->GetADC();
               mult = GetNPeaksHRS(&ICTimeVec, &ICVec, adc->data());

               for (auto tVal : ICTimeVec) {
                  time.push_back(tVal);
                  htime->Fill(tVal);
               }

               for (auto eVal : ICVec) {
                  energy.push_back(eVal);
                  henergy->Fill(eVal);
               }

               hmultiplicity->Fill(mult);
               for (auto i = 0; i < adc->size(); i++)
                  hwaveform->SetBinContent(i, adc->at(i));
            }
         }
         outtree->Fill();
      } // Events
      file->Close();
   } // Files

   std::cout << " Writing tree ..."
             << "\n";
   outfile->cd();
   outtree->Write();
   std::cout << " Finished writing tree !"
             << "\n";
   outfile->Close();

   TCanvas *c = new TCanvas();
   c->Divide(2, 2);
   c->cd(1);
   hwaveform->Draw("histo");
   c->cd(2);
   hmultiplicity->Draw("histo");
   c->cd(3);
   henergy->Draw("histo");
   c->cd(4);
   htime->Draw("histo");
}
