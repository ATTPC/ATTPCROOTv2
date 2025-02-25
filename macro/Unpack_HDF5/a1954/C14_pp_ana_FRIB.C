Double_t GetNPeaksHRS(std::vector<Int_t> *timeMax, std::vector<Float_t> *adcMax, double *adc_test)
{
   TSpectrum *s = new TSpectrum();
   Double_t dest[2048];
   // Int_t nfound = s->Search(h1_test,2," ",0.25);//2 and 0.15
   Int_t nfound;
   nfound = s->SearchHighRes(adc_test, dest, 2048, 10, 40, kFALSE, 1, kFALSE, 1);
   // nfound = s->SearchHighRes(adc_test, dest, 512, 2, 2, kTRUE, 3, kTRUE, 3);

   for (auto iPeak = 0; iPeak < nfound; ++iPeak) {

      Int_t time = (Int_t)(ceil((s->GetPositionX())[iPeak]));
      timeMax->push_back(time);
      adcMax->push_back(adc_test[time]);
   }

   delete s;
   return nfound;
}

void C14_pp_ana_FRIB(TString fileName = "run_0120")
{

   TString baseDir = "/media/david/cd93e27e-bbe0-4296-a341-7957f4adbda6/";
   TString inputFile = baseDir + fileName + ".root";


   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   std::vector<TString> files{inputFile};
   TString filesuffix = ".root";

   TH1F *hwaveform = new TH1F("waveform", "waveform", 2048, 0, 2047);
   TH1F *hmultiplicity = new TH1F("multiplicity", "multiplicity", 10, 0, 10);
   TH1F *henergy = new TH1F("energy", "energy", 4096, 0, 4095);
   TH1F *henergy_2 = new TH1F("energy_2", "energy_2", 4096, 0, 4095);
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

      TFile *file = new TFile((iFile).Data(), "READ");
      fileNames = (iFile).Data();
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
               //std::cout<<"Mult : "<<mult<<"\n";
               for (auto tVal : ICTimeVec) {
                  time.push_back(tVal);
                  htime->Fill(tVal);
                 // std::cout<<"Time: " << tVal << "\n";
               }

               for (auto eVal : ICVec) {
                  energy.push_back(eVal);
                  if (mult == 1 || mult == 2 || 3 )
                  henergy->Fill(eVal);
                  //std::cout<<"Energy: " << eVal << "\n";
               }

               hmultiplicity->Fill(mult);
               for (auto i = 0; i < adc->size(); i++)
                  hwaveform->SetBinContent(i, adc->at(i));
            }
            
               if(mult == 1)
               henergy_2->Fill(hwaveform->GetMaximum());


               if(mult == 2){
                  double maxY1 = -std::numeric_limits<double>::infinity();
                  double maxY2 = -std::numeric_limits<double>::infinity();
                  for (int bin = 1; bin <= hwaveform->GetNbinsX(); ++bin) {
                     double binContent = hwaveform->GetBinContent(bin);
                     if (binContent > maxY1) {
                      maxY2 = maxY1;
                      maxY1 = binContent;
                     } else if (binContent > maxY2) {
                      maxY2 = binContent;
                     }
                  }

                  henergy_2->Fill(maxY1);
                  henergy_2->Fill(maxY2);
               }

               if(mult == 3){
                  double maxY1 = -std::numeric_limits<double>::infinity();
                  double maxY2 = -std::numeric_limits<double>::infinity();
                  double maxY3 = -std::numeric_limits<double>::infinity();
                  for (int bin = 1; bin <= hwaveform->GetNbinsX(); ++bin) {
                     double binContent = hwaveform->GetBinContent(bin);
                     if (binContent > maxY1) {
                      maxY3 = maxY2;
                      maxY2 = maxY1;
                      maxY1 = binContent;
                     } else if (binContent > maxY2) {
                       maxY3 = maxY2;
                       maxY2 = binContent;
                     } else if (binContent > maxY3) {
                    maxY3 = binContent;
                     }
                  }

                  henergy_2->Fill(maxY1);
                  henergy_2->Fill(maxY2);
                  henergy_2->Fill(maxY3);
               }
         }


         
         //henergy_2->Fill(hwaveform->GetMaximum());
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
   //henergy_2->SetLineColor(kRed);
   //henergy_2->Draw("histo SAME");
   c->cd(4);
   htime->Draw("histo");
}
