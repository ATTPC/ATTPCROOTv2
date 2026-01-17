{  
  TGraph* ReadKinematics(TString kineFile)
  {
  Double_t *ThetaCMS = new Double_t[20000];
  Double_t *ThetaLabRec = new Double_t[20000];
  Double_t *EnerLabRec = new Double_t[20000];
  Double_t *ThetaLabSca = new Double_t[20000];
  Double_t *EnerLabSca = new Double_t[20000];
  Double_t *MomLabRec = new Double_t[20000];

  //  std::ifstream *kineStr = new std::ifstream(kineFile.Data());
  std::ifstream kineStr = std::ifstream(kineFile.Data());
  std::string line;
  Int_t numKin = 0;
  vector<vector<double>> vkine(5,vector<double>(0));
  
  if (!kineStr.fail()){
    getline(kineStr, line);
    while (!kineStr.eof()){
      kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >>
	ThetaLabSca[numKin] >> EnerLabSca[numKin];
      vkine[0].push_back(ThetaCMS[numKin]);
      vkine[1].push_back(ThetaLabRec[numKin]);
      vkine[2].push_back(EnerLabRec[numKin]);
      vkine[3].push_back(ThetaLabSca[numKin]);
      vkine[4].push_back(EnerLabSca[numKin]);
      numKin++;
    }
  }
  else if (kineStr.fail()){
    std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;
  }
  sort(vkine.begin(),vkine.end(),[](const vector<double> &a,const vector<double>& b){return a[3]<b[3];});
  //  TGraph *kine = new TGraph(numKin, ThetaLabRec, EnerLabRec);
  TGraph *kine = new TGraph(numKin, vkine.at(3).data(), vkine.at(4).data());
  return kine;
}

  TChain *fc = new TChain("tree");
  TString ss;
  for(int run_num = 4019; run_num <= 4019; run_num ++){
    ss.Form("./transfer_data/simple_%d.root",run_num);
    fc->Add(ss.Data());
  }

  // Kinematic lines.
  TGraph *kine_dd = ReadKinematics("./kineFiles/11Be_dd_gs.txt");
  TGraph *kine_d3He = ReadKinematics("./kineFiles/11Be_d3He_gs.txt");
  TGraph *kine_d3HeEx2_2 = ReadKinematics("./kineFiles/11Be_d3He_Ex1_5.txt");
  TGraph *kine_d3HeEx2_7 = ReadKinematics("./kineFiles/11Be_d3He_Ex6.txt");
  TGraph *kine_d3He_tt = ReadKinematics("./kineFiles/11Be_dp_gs.txt");
  //   TGraph *kine_12C12C = ReadKinematics("./kineFiles/17N_12C12C_gs.txt");

  fc->Draw("kineE_d[0]:thetaLab[0]>>h1(180,0,180,100,0,10)","","colz");
  kine_dd->Draw("same");

}

