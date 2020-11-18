

void PIDS800Corr(int runNumberS800){


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



  TString digiFileName = TString::Format("/mnt/analysis/e18027/rootS800/cal/run-%04d-00.root", runNumberS800);

  TFile* file = new TFile(digiFileName,"READ");

  TTree* tree = (TTree*) file -> Get("caltree");
  Int_t nEvents = tree -> GetEntries();

  S800Calc *s800cal = new S800Calc();
  TBranch *bS800cal = tree->GetBranch("s800calc");
  bS800cal->SetAddress(&s800cal);
 


  TFile* outfile;
  TString outFileName = TString::Format("../rootPID/PIDCorr_run%04d.root", runNumberS800);
  outfile   = TFile::Open(outFileName.Data(),"recreate");

  //auto c1 = new TCanvas("c1", "c1", 800, 800);
  //auto c2 = new TCanvas("c2", "c2", 800, 800);
  TH2D *XfpObj_Obj = new TH2D ("XfpObj_Obj", "XfpObj_Obj",500,-70,-20,600,250,280);//PID1
  TH2D *ICSum_Obj = new TH2D ("ICSum_Obj", "ICSum_Obj",500,-70,-20,1000,50,750);//PID2
  TH2D *CRDCX_Obj = new TH2D ("CRDCX_Obj","CRDCX_Obj",500,-70,-20,1000,-500,500);


  Float_t Xf=0,Obj=0,Xf_Obj=0;
  Double_t ICSum=0;
  Double_t CRDC1_X=0, CRDC2_X=0;
  Double_t afp=0;
  Double_t ObjCorr=0;
  Double_t Xf_ObjCorr=0.;
  Int_t CondMTDCObj = 0;
  Int_t CondMTDCXfObj = 0;

  TTree *atree = new TTree("atree","new TTree");
  atree->Branch("Xf",&Xf);
  atree->Branch("Obj",&Obj);
  atree->Branch("Xf_Obj",&Xf_Obj);
  atree->Branch("ICSum",&ICSum);
  atree->Branch("CRDC1_X",&CRDC1_X);
  atree->Branch("CRDC2_X",&CRDC2_X);
  atree->Branch("afp",&afp);
  atree->Branch("ObjCorr",&ObjCorr);
  atree->Branch("CondMTDCXfObj",&CondMTDCXfObj);
  atree->Branch("CondMTDCObj",&CondMTDCObj);

  //TCutG *fcutPID1;

  std::cout<<nEvents<<std::endl;

  for(Int_t i=0;i<nEvents;i++){
    s800cal->Clear();
    bS800cal->GetEntry(i);
    // std::cout<<i<<std::endl;
    ObjCorr=0.;
    ICSum=0.;

    Double_t S800_timeRf = s800cal->GetMultiHitTOF()->GetFirstRfHit();
    Double_t S800_timeE1up = s800cal->GetMultiHitTOF()->GetFirstE1UpHit();
    Double_t S800_timeE1down = s800cal->GetMultiHitTOF()->GetFirstE1DownHit();
    Double_t S800_timeE1 = sqrt( (corrGainE1up*S800_timeE1up) * (corrGainE1down*S800_timeE1down) );
    Double_t S800_timeXf = s800cal->GetMultiHitTOF()->GetFirstXfHit();
    Double_t S800_timeObj = s800cal->GetMultiHitTOF()->GetFirstObjHit();

    Double_t S800_x0 = s800cal->GetCRDC(0)->GetX();
    Double_t S800_x1 = s800cal->GetCRDC(1)->GetX();
    Double_t S800_y0 = s800cal->GetCRDC(0)->GetY();
    Double_t S800_y1 = s800cal->GetCRDC(1)->GetY();

    Double_t S800_E1up = s800cal->GetSCINT(0)->GetDEup();
    Double_t S800_E1down = s800cal->GetSCINT(0)->GetDEdown();

   // Double_t S800_tof = S800_timeObj - S800_timeE1;
    //Double_t XfObj_tof = S800_timeXf - S800_timeObj;

    Double_t S800_afp = atan( (S800_x1-S800_x0)/1073. );
    Double_t S800_bfp = atan( (S800_y1-S800_y0)/1073. );
    //Double_t S800_tofCorr = S800_tof + x0_corr_tof*S800_x0 + afp_corr_tof*S800_afp;// - rf_offset;
    //Double_t S800_dE = s800cal->GetSCINT(0)->GetDE();//check if is this scint (0)
    //Double_t S800_dE = sqrt( (corrGainE1up*S800_E1up) * (corrGainE1down* S800_E1down ) );
    //Double_t S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);

    Double_t ObjCorr1C1 = 100.; //70//100.
    Double_t ObjCorr1C2 = 0.021; //0.0085//0.021

//----------- New 10/01 -------------------------------------
    CondMTDCObj = 0;
    CondMTDCXfObj = 0;
    vector<Float_t> S800_timeMTDCObj = s800cal->GetMultiHitTOF()->GetMTDCObj();
    vector<Float_t> S800_timeMTDCXf = s800cal->GetMultiHitTOF()->GetMTDCXf();
    Float_t S800_timeObjSelect=-999;
    Float_t S800_timeXfSelect=-999;

    for(int k=0; k<S800_timeMTDCXf.size(); k++){
    	if(S800_timeMTDCXf.at(k)>140 && S800_timeMTDCXf.at(k)<205) S800_timeXfSelect=S800_timeMTDCXf.at(k);//150-315
    }
    for(int k=0; k<S800_timeMTDCObj.size(); k++){
    	if(S800_timeMTDCObj.at(k)>-120 && S800_timeMTDCObj.at(k)<50) S800_timeObjSelect=S800_timeMTDCObj.at(k);//-60-30
    }

    Double_t XfObj_tof = S800_timeXfSelect - S800_timeObjSelect;
    if(S800_timeObjSelect!=-999){
	CondMTDCObj=1;
        Obj=S800_timeObjSelect;
	ObjCorr = Obj + ObjCorr1C1*S800_afp + ObjCorr1C2*S800_x0;
    }
    if(S800_timeXfSelect!=-999 && S800_timeObjSelect!=-999) {
    	XfObj_tof=S800_timeXfSelect-S800_timeObjSelect;
	Xf=S800_timeXfSelect;
	Xf_Obj=XfObj_tof;
	CondMTDCXfObj=1;
    }
    Double_t S800_ICSum = s800cal->GetIC()->GetSum();
//----------- New 10/01 -------------------------------------


    //std::cout<<S800_tofCorr<<"  "<<S800_dECorr<<"  "<<XfObj_tof<<std::endl;

    if(std::isnan(ObjCorr)==1 || std::isnan(XfObj_tof)==1 || std::isnan(S800_ICSum)==1) continue;


      //if(CondMTDCXfObj)XfpObj_Obj->Fill(S800_timeObjSelect,XfObj_tof);
      if(CondMTDCXfObj)XfpObj_Obj->Fill(ObjCorr,XfObj_tof);
      //if(CondMTDCObj)ICSum_Obj->Fill(S800_timeObjSelect,S800_ICSum);
      if(CondMTDCObj)ICSum_Obj->Fill(ObjCorr,S800_ICSum);

    //std::cout<<S800_timeObjSelect<<"  "<<XfObj_tof<<" "<<S800_ICSum<<std::endl;

     ICSum=S800_ICSum;
     CRDC1_X=S800_x0;
     CRDC2_X=S800_x1;
     afp=S800_afp;
     

     CRDCX_Obj->Fill(ObjCorr,CRDC1_X);

     atree->Fill();

  }//event for loop
  //atree->Write();
  //c1->cd();
  //ICSum_Obj->Draw("colz");
  ICSum_Obj->Write();
  //c2->cd();
  //XfpObj_Obj->Draw("colz");
  XfpObj_Obj->Write();
  //CRDCX_Obj->Draw("colz");
  atree->Write();
outfile->Close();

}
