void C14_xs()
{

   // DWBA calculations
   Double_t sigmaDWBA0 = 0;
   Double_t sigmaDWBA1 = 0;
   Double_t sigmaDWBA2 = 0;
   Double_t altXS = 0;
   Double_t angle = 0;
   TGraphErrors *gDWBA0 = new TGraphErrors();
   gDWBA0->SetMarkerStyle(20);
   gDWBA0->SetMarkerSize(1.5);
   gDWBA0->SetLineWidth(3);

   std::ifstream dwbaFile("DWBA14C.Xsec.txt");
   std::string linebuff;
   // Read header and zero degree
   // for(auto i=0;i<5;++i)
   //  std::getline(dwbaFile, linebuff);

   while (!dwbaFile.eof()) {
      std::getline(dwbaFile, linebuff);
      std::istringstream iss(linebuff);
      // iss >> angle >> sigmaDWBA0 >> sigmaDWBA1 >> sigmaDWBA2;
      iss >> angle >> sigmaDWBA0;
      gDWBA0->SetPoint(gDWBA0->GetN(), angle, sigmaDWBA0);

      // gDWBA1->SetPoint(gDWBA1->GetN(), angle, sigmaDWBA1);
      // gDWBA2->SetPoint(gDWBA2->GetN(), angle, sigmaDWBA2);
   }

   dwbaFile.close();

   Double_t cnt_2_1[8] = {37.0, 57.1841, 29.4875, 64.0993, 81.1007, 102.892, 133.756, 207.129};
   // Double_t cnt_2_1[7] = {};
   Double_t ang_2_1[8] = {15.0, 25.0, 35.0, 45.0, 55.0, 65.0, 75.0, 85.0};

   Double_t xs_2_1[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   for (auto i = 0; i < 8; ++i) {
      ang_2_1[i] = ang_2_1[i];
      xs_2_1[i] = cnt_2_1[i] / TMath::Sin(TMath::DegToRad() * ang_2_1[i]);
   }

   TGraph *gxs_2_1 = new TGraph(8, ang_2_1, xs_2_1);

   // gDWBA0->Draw("AL");
   gxs_2_1->Draw("A*");
}
