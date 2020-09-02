void findEvents(int tpc_run_num=118)
{
  TChain tpc_tree("cbmsim");
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));
  
  TTreeReader reader(&tpc_tree);
  TTreeReaderValue<TClonesArray> ransac(reader, "ATRansac");

  TH2F vertex_z("vertex_z", ";z (mm;opening angle (deg)", 150, 0, 1500, 720, 0, 360);
  int tpc_nevents = tpc_tree.GetEntries();
  for(int i = 0; i < tpc_nevents; ++i)
  {
    reader.Next();
    if(ransac -> GetEntries() > 0)
    {
      auto atransac = (ATRANSACN::ATRansac*) (ransac -> At(0));
      auto ransac_ary = atransac -> GetTrackCand();
      
      int num_valid_tracks = 0;
      std::vector<TVector3> track_vec;
      for(int i = 0; i < ransac_ary.size(); ++i)
      {
        double angle = ransac_ary[i].GetAngleZAxis()*TMath::RadToDeg();
        if(2 < angle && ransac_ary[i].GetHitArray()->size() > 30) 
	{
          auto fitPar = ransac_ary[i].GetFitPar();
	  track_vec.emplace_back(fitPar[1], fitPar[3], 0);
	}
      }
      if(track_vec.size() == 2)
        vertex_z.Fill(atransac -> GetVertexMean().z(), fabs(track_vec[0].Angle(track_vec[1])*TMath::RadToDeg()));
    }
  }
  vertex_z.Draw("colz");
  gPad -> WaitPrimitive("CUTG", "[CUTG]");
}

void testCorrelation(int nscl_run_num=310, int tpc_run_num=118)
{
  TChain nscl_tree("tr");
  nscl_tree.Add(TString::Format("/mnt/analysis/e12014/HiRAEVT/Calibrated/run-%02d-aligned.root", nscl_run_num));
  TChain tpc_tree("cbmsim");
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));
  
  TTreeReader reader(&nscl_tree);
  TTreeReaderValue<TClonesArray> eventArray(reader, "ATEventH");
  TTreeReaderValue<std::vector<double>> beam_ic(reader, "IonCb_dE");

  TGraph timeVsRatio;//, hist;//
  TH2F hist("hist", ";ADC Ch 2;Aux channel max.",100,2000,3000,100,700,1200);
  int nscl_nevents = nscl_tree.GetEntries();
  int tpc_nevents = tpc_tree.GetEntries();
  nscl_tree.AddFriend(&tpc_tree);
  cout << nscl_nevents << " " << tpc_nevents << endl;
  for(int i = 0; i < ((nscl_nevents > tpc_nevents)? tpc_nevents : nscl_nevents); ++i)
  {
    reader.Next();
    auto event = (ATEvent*) eventArray -> At(0);
    auto auxPadArray = event -> GetAuxPadArray();
    for(auto auxpad : *auxPadArray)
    {
      if(auxpad.GetAuxName().compare(std::string("IonCb_34")) == 0)
      {
        auto adc = auxpad.GetADC();
        auto max = std::max_element(adc, adc+512);
        if(beam_ic->size() > 0) 
        {
	  hist.Fill(beam_ic->at(2), *max);
	  //hist.SetPoint(hist.GetN(), beam_ic->at(2), *max);
	  timeVsRatio.SetPoint(timeVsRatio.GetN(), i, beam_ic->at(2)/(*max));
	}
      }
    }
    std::cout << "Event " << i << "\r" << std::flush;
  }
  std::cout << std::endl;
  TCanvas c1;
  hist.Draw("colz");
  TCanvas c2;
  timeVsRatio.Draw("AP");
  gPad -> WaitPrimitive("CUTG", "[CUTG]");

}


void testCorrelationTracks(int nscl_run_num=310, int tpc_run_num=118)
{
  TChain nscl_tree("tr");
  nscl_tree.Add(TString::Format("/mnt/analysis/e12014/HiRAEVT/Calibrated/run-%02d-aligned.root", nscl_run_num));
  TChain tpc_tree("cbmsim");
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));
  
  TTreeReader reader(&nscl_tree);
  TTreeReaderValue<TClonesArray> ransac(reader, "ATRansac");
  TTreeReaderValue<TVector3> beam_ic(reader, "BeamOnATTPC");
  //TTreeReaderValue<double> beam_ic(reader, "Truncated_dE_to_U6");

  TH2F hist("hist", "", 5000,-500,500,5000,-500,500);
  int nscl_nevents = nscl_tree.GetEntries();
  int tpc_nevents = tpc_tree.GetEntries();
  nscl_tree.AddFriend(&tpc_tree);
  for(int i = 0; i < ((nscl_nevents > tpc_nevents)? tpc_nevents : nscl_nevents); ++i)
  {
    reader.Next();
    if(ransac -> GetEntries() > 0)
    {
      //auto ransac_ary = ((ATRANSACN::ATRansac*) (ransac -> At(0))) -> GetTrackCand();
      //if(ransac_ary.size() == 1)
      
      double sum_energy = 0;
      auto atransac = (ATRANSACN::ATRansac*) (ransac -> At(0));
      auto ransac_ary = atransac -> GetTrackCand();
      
      int num_valid_tracks = 0;
      for(int i = 0; i < ransac_ary.size(); ++i)
      {
        double angle = ransac_ary[i].GetAngleZAxis()*TMath::RadToDeg();
        if(2 < angle && ransac_ary[i].GetHitArray()->size() > 30) 
          ++num_valid_tracks;
      }
      if(num_valid_tracks == 2) //for(int i = 0; i < ransac_ary.size(); ++i)
      {
        hist.Fill(atransac -> GetVertexMean().x(), beam_ic->x());
      	//std::cout << ransac_ary[i].GetGeoQEnergy() << std::endl;
        //auto parFit = ransac_ary[i].GetFitPar();
        // calculate beam angle
        //if(ransac_ary[i].GetAngleZAxis()*TMath::RadToDeg() < 10) sum_energy += ransac_ary[i].GetGeoQEnergy();
        //hist.Fill(beam_ic->Theta()*TMath::RadToDeg(), ransac_ary[i].GetAngleZAxis()*TMath::RadToDeg());//, ransac_ary[i].GetGeoQEnergy());
      }
      //hist.Fill(*beam_ic, sum_energy);
      
    }
    std::cout << "Event " << i << "\r" << std::flush;
  }
  std::cout << std::endl;
  hist.Draw("colz");
  gPad -> WaitPrimitive("CUTG", "[CUTG]");

}


