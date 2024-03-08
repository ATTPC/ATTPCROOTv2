void plot_C15pd()
{

   // Histograms
   auto *hex = new TH1F("hex", "hex", 600, -5, 55);
   auto *hexcorr = new TH1F("hexcorr", "hexcorr", 600, -5, 55);
   auto *hkine = new TH2F("hkine", "hkine", 720, 0, 179, 1000, 0, 200.0);
   auto *hsigmaCM0 = new TH1F("hsigmaCM0", "hsigmaCM0", 360, 0, 360);
   auto *hsigmaCM1 = new TH1F("hsigmaCM1", "hsigmaCM1", 360, 0, 360);
   auto *hsigmaCM2 = new TH1F("hsigmaCM2", "hsigmaCM2", 360, 0, 360);
   auto *hsigmaCM3 = new TH1F("hsigmaCM3", "hsigmaCM3", 360, 0, 360);

   TGraphErrors *gsigmaLab0 = new TGraphErrors();
   gsigmaLab0->SetMarkerStyle(21);
   gsigmaLab0->SetMarkerSize(1.5);
   gsigmaLab0->SetMarkerColor(kRed);
   TGraphErrors *gsigmaCM0 = new TGraphErrors();
   gsigmaCM0->SetMarkerStyle(21);
   gsigmaCM0->SetMarkerSize(1.5);
   gsigmaCM0->SetMarkerColor(kRed);
   TGraphErrors *gsigmaCM1 = new TGraphErrors();
   gsigmaCM1->SetMarkerStyle(21);
   gsigmaCM1->SetMarkerSize(1.5);
   gsigmaCM1->SetMarkerColor(kRed);
   TGraphErrors *gsigmaCM2 = new TGraphErrors();
   gsigmaCM2->SetMarkerStyle(21);
   gsigmaCM2->SetMarkerSize(1.5);
   gsigmaCM2->SetMarkerColor(kRed);
   TGraphErrors *gsigmaCM3 = new TGraphErrors();
   gsigmaCM3->SetMarkerStyle(21);
   gsigmaCM3->SetMarkerSize(1.5);
   gsigmaCM3->SetMarkerColor(kRed);

   // DWBA calculations
   Double_t sigmaDWBA0 = 0;
   Double_t sigmaDWBA1 = 0;
   Double_t sigmaDWBA2 = 0;
   Double_t sigmaDWBA3 = 0;
   Double_t altXS = 0;
   Double_t angle = 0;
   TGraphErrors *gDWBA0 = new TGraphErrors();
   gDWBA0->SetMarkerStyle(20);
   gDWBA0->SetMarkerSize(1.5);
   gDWBA0->SetLineWidth(3);
   TGraphErrors *gDWBA1 = new TGraphErrors();
   gDWBA1->SetMarkerStyle(20);
   gDWBA1->SetMarkerSize(1.5);
   gDWBA1->SetLineWidth(3);
   TGraphErrors *gDWBA2 = new TGraphErrors();
   gDWBA2->SetMarkerStyle(20);
   gDWBA2->SetMarkerSize(1.5);
   gDWBA2->SetLineWidth(3);
   TGraphErrors *gDWBA3 = new TGraphErrors();
   gDWBA3->SetMarkerStyle(20);
   gDWBA3->SetMarkerSize(1.5);
   gDWBA3->SetLineWidth(3);

   std::ifstream dwbaFile("calculations/21.15gspd_y2");
   std::ifstream dwbaFile2("calculations/21.15isopd_y2");

   std::string linebuff;

   while (!dwbaFile.eof()) {
      std::getline(dwbaFile, linebuff);
      std::istringstream iss(linebuff);
      iss >> angle >> sigmaDWBA0 >> sigmaDWBA1;
      gDWBA0->SetPoint(gDWBA0->GetN(), angle, sigmaDWBA0);
   }

   while (!dwbaFile2.eof()) {
      std::getline(dwbaFile2, linebuff);
      std::istringstream iss(linebuff);
      iss >> angle >> sigmaDWBA1 >> sigmaDWBA2;
      gDWBA1->SetPoint(gDWBA1->GetN(), angle, sigmaDWBA1);
   }

   dwbaFile.close();
   dwbaFile2.close();

   /*std::ifstream dwbaFile("calculations/DWBA_XS_15Cpd.txt");

    std::string linebuff;
    //Read header and zero degree
    for(auto i=0;i<5;++i)
      std::getline(dwbaFile, linebuff);

    while (!dwbaFile.eof()) {
       std::getline(dwbaFile, linebuff);
       std::istringstream iss(linebuff);
       iss >> angle >> sigmaDWBA0 >> sigmaDWBA1 >> sigmaDWBA2 >> sigmaDWBA3;
       gDWBA0->SetPoint(gDWBA0->GetN(), angle, sigmaDWBA0);
       gDWBA1->SetPoint(gDWBA1->GetN(), angle, sigmaDWBA1);
       gDWBA2->SetPoint(gDWBA2->GetN(), angle, sigmaDWBA2);
       gDWBA3->SetPoint(gDWBA3->GetN(), angle, sigmaDWBA3);
    }

    dwbaFile.close(); */

   TFile *file = new TFile("C15_pd_analysis.root", "READ");
   if (!file || file->IsZombie()) {
      std::cerr << "Error: cannot open file 'C15_pd_analysis.root'." << std::endl;
      return;
   }

   TTree *tree = dynamic_cast<TTree *>(file->Get("output"));
   if (!tree) {
      std::cerr << "Error: cannot get tree from file." << std::endl;
      file->Close();
      return;
   }

   auto numEvents = tree->GetEntries();
   std::cout << "Number of events: " << numEvents << std::endl;

   Double_t _theta = 0.0, _energy = 0.0, _exEnergy = 0.0, _exEnergyCorr = 0.0, _zVertex = 0.0, _thetacm = 0.0;
   tree->SetBranchAddress("_theta", &_theta);
   tree->SetBranchAddress("_energy", &_energy);
   tree->SetBranchAddress("_exEnergy", &_exEnergy);
   tree->SetBranchAddress("_exEnergyCorr", &_exEnergyCorr);
   tree->SetBranchAddress("_zVertex", &_zVertex);
   tree->SetBranchAddress("_thetacm", &_thetacm);

   for (auto i = 0; i < numEvents; i++) {
      tree->GetEntry(i);
      hex->Fill(_exEnergy);
      hexcorr->Fill(_exEnergyCorr);
      hkine->Fill(_theta, _energy);
      if (_exEnergyCorr > -0.4 && _exEnergyCorr < 0.35)
         hsigmaCM0->Fill(_thetacm);
      if (_exEnergyCorr > 0.35 && _exEnergyCorr < 1.3)
         hsigmaCM1->Fill(_thetacm);
      if (_exEnergyCorr > 2.4 && _exEnergyCorr < 3.5)
         hsigmaCM2->Fill(_thetacm);
      if (_exEnergyCorr > 3.7 && _exEnergyCorr < 5.0)
         hsigmaCM3->Fill(_thetacm);
   }

   for (auto i = 1; i < hsigmaCM0->GetNbinsX(); i++) {
      hsigmaCM0->SetBinContent(i, hsigmaCM0->GetBinContent(i) /
                                     TMath::Sin(TMath::DegToRad() * hsigmaCM0->GetBinCenter(i)));
      hsigmaCM1->SetBinContent(i, hsigmaCM1->GetBinContent(i) /
                                     TMath::Sin(TMath::DegToRad() * hsigmaCM1->GetBinCenter(i)));
      hsigmaCM2->SetBinContent(i, hsigmaCM2->GetBinContent(i) /
                                     TMath::Sin(TMath::DegToRad() * hsigmaCM2->GetBinCenter(i)));
      hsigmaCM3->SetBinContent(i, hsigmaCM3->GetBinContent(i) /
                                     TMath::Sin(TMath::DegToRad() * hsigmaCM3->GetBinCenter(i)));
   }

   hsigmaCM0->Rebin(2);
   hsigmaCM1->Rebin(2);
   hsigmaCM2->Rebin(2);
   hsigmaCM3->Rebin(2);

   hsigmaCM0->Scale(1.0 / 20.0);
   hsigmaCM1->Scale(1.0 / 20.0);
   hsigmaCM2->Scale(1.0 / 20.0);
   hsigmaCM3->Scale(1.0 / 20.0);

   for (auto i = 1; i < hsigmaCM0->GetNbinsX(); i++) {
      gsigmaCM0->SetPoint(gsigmaCM0->GetN(), hsigmaCM0->GetBinCenter(i), hsigmaCM0->GetBinContent(i));
      gsigmaCM0->SetPointError(gsigmaCM0->GetN() - 1, 0.0, hsigmaCM0->GetBinError(i));
   }

   for (auto i = 1; i < hsigmaCM1->GetNbinsX(); i++) {
      gsigmaCM1->SetPoint(gsigmaCM1->GetN(), hsigmaCM1->GetBinCenter(i), hsigmaCM1->GetBinContent(i));
      gsigmaCM1->SetPointError(gsigmaCM1->GetN() - 1, 0.0, hsigmaCM1->GetBinError(i));
   }

   for (auto i = 1; i < hsigmaCM2->GetNbinsX(); i++) {
      gsigmaCM2->SetPoint(gsigmaCM2->GetN(), hsigmaCM2->GetBinCenter(i), hsigmaCM2->GetBinContent(i));
      gsigmaCM2->SetPointError(gsigmaCM2->GetN() - 1, 0.0, hsigmaCM2->GetBinError(i));
   }

   for (auto i = 1; i < hsigmaCM3->GetNbinsX(); i++) {
      gsigmaCM3->SetPoint(gsigmaCM3->GetN(), hsigmaCM3->GetBinCenter(i), hsigmaCM3->GetBinContent(i));
      gsigmaCM3->SetPointError(gsigmaCM3->GetN() - 1, 0.0, hsigmaCM3->GetBinError(i));
   }

   file->Close();

   TFile *filePS = new TFile("PhaseSpace_16Cdt15C.root", "READ");
   if (!filePS || filePS->IsZombie()) {
      std::cerr << "Error: cannot open file 'PhaseSpace_16Cdt15C.root'." << std::endl;
      return;
   }

   auto h_PS = (TH1F *)filePS->Get("h_PS");
   h_PS->Scale(.05);

   TCanvas *c1 = new TCanvas("c1", "c1", 800, 600);
   hex->Draw();

   TCanvas *c2 = new TCanvas("c2", "c2", 800, 600);
   hexcorr->Draw();
   // h_PS->Draw("same");

   TCanvas *c4 = new TCanvas("c4", "c4", 800, 600);
   hkine->Draw("colz");

   TCanvas *c3 = new TCanvas("c3", "c3", 800, 600);
   c3->Divide(2, 2);
   c3->SetLogy(1);
   c3->cd(1);
   hsigmaCM0->Draw("pe");
   hsigmaCM0->SetMarkerStyle(20);
   hsigmaCM0->SetMarkerSize(1.0);
   gPad->SetLogy();
   c3->cd(2);
   hsigmaCM1->Draw();
   hsigmaCM1->Draw("pe");
   hsigmaCM1->SetMarkerStyle(20);
   hsigmaCM1->SetMarkerSize(1.0);
   gPad->SetLogy();
   c3->cd(3);
   hsigmaCM2->Draw();
   hsigmaCM2->Draw("pe");
   hsigmaCM2->SetMarkerStyle(20);
   hsigmaCM2->SetMarkerSize(1.0);
   gPad->SetLogy();
   c3->cd(4);
   hsigmaCM3->Draw();
   hsigmaCM3->Draw("pe");
   hsigmaCM3->SetMarkerStyle(20);
   hsigmaCM3->SetMarkerSize(1.0);
   gPad->SetLogy();

   TCanvas *c5 = new TCanvas("c5", "c5", 800, 600);
   c5->SetLogy(1);
   gDWBA0->Draw("al");
   gsigmaCM0->Draw("p");

   TCanvas *c6 = new TCanvas("c6", "c6", 800, 600);
   c6->SetLogy(1);
   gDWBA1->Draw("al");
   gsigmaCM1->Draw("p");

   TCanvas *c7 = new TCanvas("c7", "c7", 800, 600);
   c7->SetLogy(1);
   gDWBA2->Draw("al");
   gsigmaCM2->Draw("p");

   TCanvas *c8 = new TCanvas("c8", "c8", 800, 600);
   c8->SetLogy(1);
   gDWBA3->Draw("al");
   gsigmaCM3->Draw("p");

   file->Close();
}
