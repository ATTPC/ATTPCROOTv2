
//void PIDS800(int runNumberS800, int runNumberATTPC){
void PIDS800(int runNumberS800){


  // Double_t x0_corr_tof = 0.101259;
  // Double_t afp_corr_tof = 1177.02;
  // Double_t afp_corr_dE = 61.7607;
  // Double_t x0_corr_dE = -0.0403;
  // Double_t rf_offset = 0.0;
  Double_t x0_corr_tof = 0.;
  Double_t afp_corr_tof = 0.;
  Double_t afp_corr_dE = 0.;
  Double_t x0_corr_dE = 0.;
  Double_t rf_offset = 0.;
  Double_t corrGainE1up = 1.;//0.6754
  Double_t corrGainE1down = 1.;



  //TString digiFileName = TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d.root", runNumberS800, runNumberATTPC);
  TString digiFileName = TString::Format("/mnt/analysis/e18008/rootS800/cal/run-%04d-00.root", runNumberS800);

  TFile* file = new TFile(digiFileName,"READ");

  TTree* tree = (TTree*) file -> Get("caltree");
  //TTree* tree = (TTree*) file -> Get("cbmsim");
  Int_t nEvents = tree -> GetEntries();

  S800Calc *s800cal = new S800Calc();
  //TBranch *bS800cal = tree->GetBranch("s800cal");
  TBranch *bS800cal = tree->GetBranch("s800calc");
  bS800cal->SetAddress(&s800cal);
  // TTreeReader reader("caltree", file);
  //TTreeReaderValue<S800Calc> *readerValueS800Calc;
  //readerValueS800Calc = new TTreeReaderValue<S800Calc>(reader, "s800calc");
  // TTreeReaderValue<S800Calc> s800Calc(reader, "s800calc");
  //S800Calc *fS800CalcBr = new S800Calc;
  //*fS800CalcBr = (S800Calc) *readerValueS800Calc->Get();

  // S800Calc s800cal;


  
  auto c1 = new TCanvas("c1", "c1", 800, 800);
  auto c2 = new TCanvas("c2", "c2", 800, 800);
  TH2D *XfpObj_Obj = new TH2D ("XfpObj_Obj", "XfpObj_Obj",500,-100,0,300,250,280);//PID1
  TH2D *ICSum_Obj = new TH2D ("ICSum_Obj", "ICSum_Obj",200,-100,0,500,0,750);//PID2

  TFile* outfile;
  TString outFileName = TString::Format("rootPID/PID_run%04d.root", runNumberS800);
  outfile   = TFile::Open(outFileName.Data(),"recreate");

  Float_t Xf=0,Obj=0,Xf_Obj=0;
  Double_t ICSum=0;

  TTree *atree = new TTree("atree","new TTree");
  atree->Branch("Xf",&Xf);
  atree->Branch("Obj",&Obj);
  atree->Branch("Xf_Obj",&Xf_Obj);
  atree->Branch("ICSum",&ICSum);

  //TCutG *fcutPID1;

  std::cout<<nEvents<<std::endl;

  for(Int_t i=0;i<nEvents;i++){
    s800cal->Clear();
    bS800cal->GetEntry(i);
    // reader.Next();
    // s800cal = *s800Calc;
    // std::cout<<i<<std::endl;

    Double_t S800_timeRf = s800cal->GetMultiHitTOF()->GetFirstRfHit();
    Double_t S800_timeE1up = s800cal->GetMultiHitTOF()->GetFirstE1UpHit();
    Double_t S800_timeE1down = s800cal->GetMultiHitTOF()->GetFirstE1DownHit();
    Double_t S800_timeE1 = sqrt( (corrGainE1up*S800_timeE1up) * (corrGainE1down*S800_timeE1down) );
    Double_t S800_timeXf = s800cal->GetMultiHitTOF()->GetFirstXfHit();
    Double_t S800_timeObj = s800cal->GetMultiHitTOF()->GetFirstObjHit();
    
//----------- New 10/01 ------------------------------------- 
    Int_t CondMTDCObj = 0;
    Int_t CondMTDCXfObj = 0;
    vector<Float_t> S800_timeMTDCObj = s800cal->GetMultiHitTOF()->GetMTDCObj();
    vector<Float_t> S800_timeMTDCXf = s800cal->GetMultiHitTOF()->GetMTDCXf();
    Float_t S800_timeObjSelect=-999;
    Float_t S800_timeXfSelect=-999;
    
    for(int k=0; k<S800_timeMTDCXf.size(); k++){
    	if(S800_timeMTDCXf.at(k)>150 && S800_timeMTDCXf.at(k)<315) S800_timeXfSelect=S800_timeMTDCXf.at(k);
    }
    for(int k=0; k<S800_timeMTDCObj.size(); k++){
    	if(S800_timeMTDCObj.at(k)>-60 && S800_timeMTDCObj.at(k)<30) S800_timeObjSelect=S800_timeMTDCObj.at(k);
    }

    Double_t XfObj_tof = S800_timeXfSelect - S800_timeObjSelect;
    if(S800_timeObjSelect!=-999){
	CondMTDCObj=1;
        Obj=S800_timeObjSelect;
    }
    if(S800_timeXfSelect!=-999 && S800_timeObjSelect!=-999) {
    	XfObj_tof=S800_timeXfSelect-S800_timeObjSelect;
	Xf=S800_timeXfSelect;
	Xf_Obj=XfObj_tof;
	CondMTDCXfObj=1;
    } 
    Double_t S800_ICSum = s800cal->GetIC()->GetSum();
//----------- New 10/01 ------------------------------------- 
 

    Double_t S800_x0 = s800cal->GetCRDC(0)->GetXfit();
    Double_t S800_x1 = s800cal->GetCRDC(1)->GetXfit();
    Double_t S800_y0 = s800cal->GetCRDC(0)->GetY();
    Double_t S800_y1 = s800cal->GetCRDC(1)->GetY();

    Double_t S800_E1up = s800cal->GetSCINT(0)->GetDEup();
    Double_t S800_E1down = s800cal->GetSCINT(0)->GetDEdown();

    Double_t S800_tof = S800_timeObj - S800_timeE1;
    //Double_t XfObj_tof = S800_timeXf - S800_timeObj;

    Double_t S800_afp = atan( (S800_x1-S800_x0)/1073. );
    Double_t S800_bfp = atan( (S800_y1-S800_y0)/1073. );
    Double_t S800_tofCorr = S800_tof + x0_corr_tof*S800_x0 + afp_corr_tof*S800_afp;// - rf_offset;
    //Double_t S800_dE = s800cal->GetSCINT(0)->GetDE();//check if is this scint (0)
    Double_t S800_dE = sqrt( (corrGainE1up*S800_E1up) * (corrGainE1down* S800_E1down ) );
    Double_t S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);


    //std::cout<<S800_tofCorr<<"  "<<S800_dECorr<<"  "<<XfObj_tof<<std::endl;
    
    if(std::isnan(S800_tofCorr)==1 || std::isnan(S800_dECorr)==1) continue;

      if(CondMTDCXfObj)XfpObj_Obj->Fill(S800_timeObjSelect,XfObj_tof);
      if(CondMTDCObj)ICSum_Obj->Fill(S800_timeObjSelect,S800_ICSum);

    std::cout<<S800_timeObjSelect<<"  "<<XfObj_tof<<" "<<S800_ICSum<<std::endl;

     ICSum=S800_ICSum;

     atree->Fill();

  }//event for loop
  atree->Write();
  c1->cd();
  ICSum_Obj->Draw("colz");
  c2->cd();
  XfpObj_Obj->Draw("colz");

outfile->Close();

}
