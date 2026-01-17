// Macro to analyse the AtRawEvents in order to determine charge thresholds, entrance and exit TBs, etc...
void analyseRawEvents()
{
   //Double_t threshold{100};
   Double_t threshold{1200};

   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   // AtMap to check if a hit belong to a big pad or small pad.
   TString scriptfile = "rcnp_map_size.xml";
   TString dir = getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + scriptfile;
   AtTpcMap *map = new AtTpcMap();
   map->ParseXMLMap(mapDir.Data());
   map->GeneratePadPlane();

   std::map<int, int> padNumToSmallPadNum;
   std::map<int, int> padNumToBigPadNum;
   int idxSmallPad{};
   int idxBigPad{};
   for (int i = 0; i < 10240; i++) {
      if (map->GetPadSize(i) == 0)
         padNumToSmallPadNum[i] = idxSmallPad++;
      else if (map->GetPadSize(i) == 1)
         padNumToBigPadNum[i] = idxBigPad++;
   }
   std::cout << " Number of small pads: " << idxSmallPad << std::endl;
   std::cout << " Number of big pads: " << idxBigPad << std::endl;

   // Histogram definitions.
   TH1F *histADC = new TH1F("histADC", "histADC", 10000, -5000, 5000);
   TH1F *histRawADC = new TH1F("histRawADC", "histRawADC", 65536, 0, 65536);

   TH1F *histADCvTB = new TH1F("histADCvTB", "histADCvTB", 512, 0, 512);

   TH2F *histADCvTB2 = new TH2F("histADCvTB2", "histADCvTB2", 512, 0, 512, 10000, -5000, 5000);
   TH2F *histRawADCvTB2 = new TH2F("histRawADCvTB2", "histRawADCvTB2", 512, 0, 512, 65536, 0, 65536);


   std::array<TH2F *, 12> histADCPerSmallPad = {new TH2F("histADCPerSmallPad1", "histADCPerSmallPad1", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad2", "histADCPerSmallPad2", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad3", "histADCPerSmallPad3", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad4", "histADCPerSmallPad4", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad5", "histADCPerSmallPad5", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad6", "histADCPerSmallPad6", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad7", "histADCPerSmallPad7", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad8", "histADCPerSmallPad8", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad9", "histADCPerSmallPad9", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad10", "histADCPerSmallPad10", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad11", "histADCPerSmallPad11", 512, 0, 512, 10000, -5000, 5000),
                                                new TH2F("histADCPerSmallPad12", "histADCPerSmallPad12", 512, 0, 512, 10000, -5000, 5000)};

/*   std::array<TH2F *, 12> histRawADCPerSmallPad = {new TH2F("histRawADCPerSmallPad1", "histRawADCPerSmallPad1", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad2", "histRawADCPerSmallPad2", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad3", "histRawADCPerSmallPad3", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad4", "histRawADCPerSmallPad4", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad5", "histRawADCPerSmallPad5", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad6", "histRawADCPerSmallPad6", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad7", "histRawADCPerSmallPad7", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad8", "histRawADCPerSmallPad8", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad9", "histRawADCPerSmallPad9", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad10", "histRawADCPerSmallPad10", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad11", "histRawADCPerSmallPad11", 512, 0, 512, 65536, 0, 65536),
                                                   new TH2F("histRawADCPerSmallPad12", "histRawADCPerSmallPad12", 512, 0, 512, 65536, 0, 65536)};
*/
   std::array<TH2F *, 8> histADCPerBigPad = {new TH2F("histADCPerBigPad1", "histADCPerBigPad1", 512, 0, 512, 10000, -5000, 5000),
                                             new TH2F("histADCPerBigPad2", "histADCPerBigPad2", 512, 0, 512, 10000, -5000, 5000),
                                             new TH2F("histADCPerBigPad3", "histADCPerBigPad3", 512, 0, 512, 10000, -5000, 5000),
                                             new TH2F("histADCPerBigPad4", "histADCPerBigPad4", 512, 0, 512, 10000, -5000, 5000),
                                             new TH2F("histADCPerBigPad5", "histADCPerBigPad5", 512, 0, 512, 10000, -5000, 5000),
                                             new TH2F("histADCPerBigPad6", "histADCPerBigPad6", 512, 0, 512, 10000, -5000, 5000),
                                             new TH2F("histADCPerBigPad7", "histADCPerBigPad7", 512, 0, 512, 10000, -5000, 5000),
                                             new TH2F("histADCPerBigPad8", "histADCPerBigPad8", 512, 0, 512, 10000, -5000, 5000)};

  /* std::array<TH2F *, 8> histRawADCPerBigPad = {new TH2F("histRawADCPerBigPad1", "histRawADCPerBigPad1", 512, 0, 512, 65536, 0, 65536),
                                                new TH2F("histRawADCPerBigPad2", "histRawADCPerBigPad2", 512, 0, 512, 65536, 0, 65536),
                                                new TH2F("histRawADCPerBigPad3", "histRawADCPerBigPad3", 512, 0, 512, 65536, 0, 65536),
                                                new TH2F("histRawADCPerBigPad4", "histRawADCPerBigPad4", 512, 0, 512, 65536, 0, 65536),
                                                new TH2F("histRawADCPerBigPad5", "histRawADCPerBigPad5", 512, 0, 512, 65536, 0, 65536),
                                                new TH2F("histRawADCPerBigPad6", "histRawADCPerBigPad6", 512, 0, 512, 65536, 0, 65536),
                                                new TH2F("histRawADCPerBigPad7", "histRawADCPerBigPad7", 512, 0, 512, 65536, 0, 65536),
                                                new TH2F("histRawADCPerBigPad8", "histRawADCPerBigPad8", 512, 0, 512, 65536, 0, 65536)};
*/
   std::array<TH2F *, 20> histADCPerPad = {new TH2F("histADCPerPad1", "histADCPerPad1", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad2", "histADCPerPad2", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad3", "histADCPerPad3", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad4", "histADCPerPad4", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad5", "histADCPerPad5", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad6", "histADCPerPad6", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad7", "histADCPerPad7", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad8", "histADCPerPad8", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad9", "histADCPerPad9", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad10", "histADCPerPad10", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad11", "histADCPerPad11", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad12", "histADCPerPad12", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad13", "histADCPerPad13", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad14", "histADCPerPad14", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad15", "histADCPerPad15", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad16", "histADCPerPad16", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad17", "histADCPerPad17", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad18", "histADCPerPad18", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad19", "histADCPerPad19", 512, 0, 512, 10000, -5000, 5000),
                                           new TH2F("histADCPerPad20", "histADCPerPad20", 512, 0, 512, 10000, -5000, 5000)};

  /* std::array<TH2F *, 20> histRawADCPerPad = {new TH2F("histRawADCPerPad1", "histRawADCPerPad1", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad2", "histRawADCPerPad2", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad3", "histRawADCPerPad3", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad4", "histRawADCPerPad4", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad5", "histRawADCPerPad5", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad6", "histRawADCPerPad6", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad7", "histRawADCPerPad7", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad8", "histRawADCPerPad8", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad9", "histRawADCPerPad9", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad10", "histRawADCPerPad10", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad11", "histRawADCPerPad11", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad12", "histRawADCPerPad12", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad13", "histRawADCPerPad13", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad14", "histRawADCPerPad14", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad15", "histRawADCPerPad15", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad16", "histRawADCPerPad16", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad17", "histRawADCPerPad17", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad18", "histRawADCPerPad18", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad19", "histRawADCPerPad19", 512, 0, 512, 65536, 0, 65536),
                                              new TH2F("histRawADCPerPad20", "histRawADCPerPad20", 512, 0, 512, 65536, 0, 65536)};
*/
   std::vector runNums = {4024};


   for (int runNum: runNums) {
      // Open the file with the AtRawEvents.
      TString unpackFileName = TString::Format("/data/ATTPCROOTv2_results/E581/UnpackerOutput/run_%04d_RawEvents.root", runNum);
      TFile *unpackFile = new TFile(unpackFileName, "READ");
      TTree *unpackTree = (TTree *)unpackFile->Get("cbmsim");
      int nUnpackEvents = unpackTree->GetEntries();
      std::cout << " Number of unpacked events in run " << runNum << ": " << nUnpackEvents << std::endl;

      // Creare the TTreeReader to read the AtTrackingEvents and simulation.
      TTreeReader unpackReader("cbmsim", unpackFile);
      TTreeReaderValue<TClonesArray> rawArray(unpackReader, "AtRawEvent");

      // Loop over events.
      for (int i = 0; i < nUnpackEvents; i++) {
         unpackReader.Next();

         // Get the AtRawEvent and extract the AtPads.
         AtRawEvent *rawEvent = (AtRawEvent *)rawArray->At(0);
         auto &pads = rawEvent->GetPads();

         // Iterate over AtPads.
         for (auto &pad: pads) {
	         for (int j = 0; j < 512; j++) {
               Double_t ADC = pad->GetADC(j);
               Double_t rawADC = pad->GetRawADC(j);
               Int_t padNum = pad->GetPadNum();

               histADC->Fill(ADC);
               histRawADC->Fill(rawADC);
               histADCvTB2->Fill(j, ADC);
               histRawADCvTB2->Fill(j, rawADC);

               if (ADC > threshold)
                  histADCvTB->Fill(j, ADC);

               histADCPerPad[padNum / 512]->Fill(padNum - (padNum / 512) * 512, ADC);
               //histRawADCPerPad[padNum / 512]->Fill(padNum - (padNum / 512) * 512, rawADC);

               if (map->GetPadSize(padNum) == 0) {
                  int smallPadNum = padNumToSmallPadNum.at(padNum);
                  histADCPerSmallPad[smallPadNum / 512]->Fill(smallPadNum - (smallPadNum / 512) * 512, ADC);
                  //histRawADCPerSmallPad[smallPadNum / 512]->Fill(smallPadNum - (smallPadNum / 512) * 512, rawADC);
               } else if (map->GetPadSize(padNum) == 1) {
                  int bigPadNum = padNumToBigPadNum.at(padNum);
                  histADCPerBigPad[bigPadNum / 512]->Fill(bigPadNum - (bigPadNum / 512) * 512, ADC);
                  //histRawADCPerBigPad[bigPadNum / 512]->Fill(bigPadNum - (bigPadNum / 512) * 512, rawADC);
               }
            }
         }
      }
      // Close files.
      unpackFile->Close();
   }

   TCanvas *c = new TCanvas();
   histADCvTB->Draw();
   histADCvTB->GetXaxis()->SetTitle("TB");
   histADCvTB->GetYaxis()->SetTitle("#Sigma ADC");

   TCanvas *c2 = new TCanvas();
   histADCvTB2->Draw("zcol");
   histADCvTB2->GetXaxis()->SetTitle("TB");
   histADCvTB2->GetYaxis()->SetTitle("ADC");

   TCanvas *c3 = new TCanvas();
   histADC->Draw();
   histADC->GetXaxis()->SetTitle("ADC");

   std::array<TCanvas *, 20> canvasPads;
   for (int i = 0; i < 20; i++) {
      canvasPads[i] = new TCanvas();
      histADCPerPad[i]->Draw("zcol");
      histADCPerPad[i]->GetXaxis()->SetTitle("Pad ID");
      histADCPerPad[i]->GetYaxis()->SetTitle("ADC");
   }

   std::array<TCanvas *, 12> canvasSmallPads;
   for (int i = 0; i < 12; i++) {
      canvasSmallPads[i] = new TCanvas();
      histADCPerSmallPad[i]->Draw("zcol");
      histADCPerSmallPad[i]->GetXaxis()->SetTitle("Small Pad ID");
      histADCPerSmallPad[i]->GetYaxis()->SetTitle("ADC");
   }

   std::array<TCanvas *, 8> canvasBigPads;
   for (int i = 0; i < 8; i++) {
      canvasBigPads[i] = new TCanvas();
      histADCPerBigPad[i]->Draw("zcol");
      histADCPerBigPad[i]->GetXaxis()->SetTitle("Big Pad ID");
      histADCPerBigPad[i]->GetYaxis()->SetTitle("ADC");
   }

   TCanvas *c4 = new TCanvas();
   histRawADCvTB2->Draw("zcol");
   histRawADCvTB2->GetXaxis()->SetTitle("TB");
   histRawADCvTB2->GetYaxis()->SetTitle("rawADC");

   TCanvas *c5 = new TCanvas();
   histRawADC->Draw();
   histRawADC->GetXaxis()->SetTitle("rawADC");

/*   std::array<TCanvas *, 20> canvasPadsRaw;
   for (int i = 0; i < 20; i++) {
      canvasPadsRaw[i] = new TCanvas();
      histRawADCPerPad[i]->Draw("zcol");
      histRawADCPerPad[i]->GetXaxis()->SetTitle("Pad ID");
      histRawADCPerPad[i]->GetYaxis()->SetTitle("rawADC");
   }

   std::array<TCanvas *, 12> canvasSmallPadsRaw;
   for (int i = 0; i < 12; i++) {
      canvasSmallPadsRaw[i] = new TCanvas();
      histRawADCPerSmallPad[i]->Draw("zcol");
      histRawADCPerSmallPad[i]->GetXaxis()->SetTitle("Small Pad ID");
      histRawADCPerSmallPad[i]->GetYaxis()->SetTitle("rawADC");
   }

   std::array<TCanvas *, 8> canvasBigPadsRaw;
   for (int i = 0; i < 8; i++) {
      canvasBigPadsRaw[i] = new TCanvas();
      histRawADCPerBigPad[i]->Draw("zcol");
      histRawADCPerBigPad[i]->GetXaxis()->SetTitle("Big Pad ID");
      histRawADCPerBigPad[i]->GetYaxis()->SetTitle("rawADC");
   }*/
}
