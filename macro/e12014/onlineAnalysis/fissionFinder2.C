void fissionFinder2(int tpc_run_num=200)
{
  TChain tpc_tree("cbmsim");
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));
  
  TTreeReader reader(&tpc_tree);
  TTreeReaderValue<TClonesArray> ransac(reader, "ATRansac");

  std::ofstream events_file("findEvents_" + std::to_string(tpc_run_num) + ".dat");
  events_file << "Event ID\n";

  int tpc_nevents = tpc_tree.GetEntries();
  TH1F *hAngle = new TH1F("hAngle", "Angular Distro Of Fragments", 20,0,45);
  
  auto numFission = 0;
  for(int i = 0; i < tpc_nevents; ++i)
  {
    reader.Next();
    if(ransac -> GetEntries() > 0)
    {
      auto ransac_ary = ((ATRANSACN::ATRansac*) (ransac -> At(0))) -> GetTrackCand();
      
      int num_valid_tracks = 0;
      vector<double> angles;
      
      for(int i = 0; i < ransac_ary.size(); ++i)
      {
        double angle = ransac_ary[i].GetAngleZAxis()*TMath::RadToDeg();
        if(2 < angle && ransac_ary[i].GetHitArray()->size() > 30)
	{
	  ++num_valid_tracks;//
	  angles.push_back(angle);
	  
	}
      }
      if(num_valid_tracks > 1)
      {
	events_file << reader.GetCurrentEntry() << std::endl;
	++numFission;
	for (auto angle : angles)
	  hAngle->Fill(angle);
      }
    }
  }
  std::cout << "Found " << numFission << " in " << tpc_nevents << std::endl;
  hAngle->Draw();
}

