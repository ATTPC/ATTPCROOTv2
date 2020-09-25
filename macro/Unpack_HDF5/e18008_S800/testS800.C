
void testS800(){

  std::string digiFileName = "/home/juan/FairRoot/ATTPCROOTv2_simon/test-runs800-48Ca-CAL-0001.root";
  TFile* file = new TFile(digiFileName.c_str(),"READ");

  TTree* tree = (TTree*) file -> Get("caltree");
  Int_t nEvents = tree -> GetEntries();

  TTreeReader reader("caltree", file);
  //TTreeReaderValue<S800Calc> *readerValueS800Calc;
  //readerValueS800Calc = new TTreeReaderValue<S800Calc>(reader, "s800calc");
  TTreeReaderValue<S800Calc> s800Calc(reader, "s800calc");
  //S800Calc *fS800CalcBr = new S800Calc;
  //*fS800CalcBr = (S800Calc) *readerValueS800Calc->Get();

  S800Calc s800cal;

  TH2D *tof_dE = new TH2D ("tof_dE", "tof_dE", 500, 0, 0, 500, 0, 0);//PID

  for(Int_t i=0;i<nEvents;i++){
    s800cal.Clear();
    reader.Next();
    s800cal = *s800Calc;
    //std::cout<<i<<std::endl;

    Double_t x0_corr_tof = 0.101259;
    Double_t afp_corr_tof = 1177.02;
    Double_t afp_corr_dE = 61.7607;
    Double_t x0_corr_dE = -0.0403;
    Double_t rf_offset = 0.0;
    Double_t S800_rf = s800cal.GetMultiHitTOF()->GetFirstRfHit();
    Double_t S800_x0 = s800cal.GetCRDC(0)->GetXfit();
    Double_t S800_x1 = s800cal.GetCRDC(1)->GetXfit();
    Double_t S800_y0 = s800cal.GetCRDC(0)->GetY();
    Double_t S800_y1 = s800cal.GetCRDC(1)->GetY();
    Double_t S800_E1up = s800cal.GetSCINT(0)->GetDEup();
    Double_t S800_E1down = s800cal.GetSCINT(0)->GetDEdown();
    Double_t S800_tof = S800_rf;//might change
    Double_t S800_afp = atan( (S800_x1-S800_x0)/1073. );
    Double_t S800_bfp = atan( (S800_y1-S800_y0)/1073. );
    Double_t S800_tofCorr = S800_tof + x0_corr_tof*S800_x0 + afp_corr_tof*S800_afp - rf_offset;
    //Double_t S800_dE = s800cal.GetSCINT(0)->GetDE();//check if is this scint (0)
    Double_t S800_dE = sqrt( (0.6754*S800_E1up) * ( 1.0 * S800_E1down ) );
    Double_t S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);

    if(std::isnan(S800_tofCorr)==1) continue;
     tof_dE->Fill(S800_tofCorr,S800_dECorr);

    std::cout<<S800_tofCorr<<"  "<<S800_tof<<"  "<<S800_dECorr<<"  "<<S800_dE<<"  "<<S800_x0<<std::endl;


  }

  tof_dE->Draw("colz");

}
