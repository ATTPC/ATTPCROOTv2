
bool cmp(std::pair<Int_t, Int_t> &a,
	 std::pair<Int_t, Int_t> &b);

void getConstPads(int tpcRun = 200)
{
  
  TChain tpc_tree("cbmsim");
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpcRun));
  
  TTreeReader reader(&tpc_tree);
  TTreeReaderValue<TClonesArray> event(reader, "AtEventH");

  //Open file and skip header
  std::ofstream padFile("data/constPads_" + std::to_string(tpcRun) + ".dat");
  if(!padFile.good())
    std::cout << "Failed to open file" << std::endl;;

  //Create pad plane and load map
  TString mapFile = "e12014_pad_mapping.xml";//"Lookup20150611.xml";
  //Set directories
  TString dir = gSystem->Getenv("VMCWORKDIR");
  TString mapDir = dir + "/scripts/"+ mapFile;

  auto fAtMapPtr = new AtTpcMap();
  fAtMapPtr->ParseXMLMap(mapDir.Data());
  fAtMapPtr->GenerateAtTpc();
  auto fPadPlane = fAtMapPtr->GetAtTpcPlane();
  double fThreshold = 0;

  //Make a map to hold how often each pad was recorded
  std::map<Int_t, Int_t> pads; // pads[padNum] = numHits

  std::cout << "Starting loop over events" << std::endl;
  //Loop through every event
  while(reader.Next())
  {
    auto eventPtr = (AtEvent*) (event->At(0));
    auto numHits = eventPtr->GetNumHits();

    for( int i = 0; i < numHits; ++i)
    {
      //Get the padNum for the hit
      AtHit hit = eventPtr->GetHitArray()->at(i);
      auto padNum = hit.GetHitPadNum();

      //If the pad hasn't been hit yet insert an element into the map
      if(pads.find(padNum) == pads.end())
	pads[padNum] = 0;

      //Increment the map for this pad
      pads.at(padNum)++;
    }
  }

  //Get total number of events and cuttoff for adding to pad plane
  auto numEvents = tpc_tree.GetEntries();
  std::cout << "There are " << numEvents << " in the tree" << std::endl;
  auto cutOff = numEvents * 0.9;

  
  //Sort the map by number of events
  std::vector<std::pair<Int_t, Int_t>> vecMap;
  for(auto&& it : pads)
    vecMap.push_back(it);
  std::sort(vecMap.begin(), vecMap.end(), cmp);

  std::cout << "Done sorting by number of hits in a pad" << std::endl;
  
  //for(auto&& it : fAtMapPtr->ATTPCPadMapInverse)
  //std::cout << it.first << std::endl;

  //Loop through and print in csv file and fill pad plane
  for (auto&& it : vecMap)
  {
    auto padRef = fAtMapPtr->GetPadRef(it.first);
    
    //Write CSV file
    padFile << it.first << "," << it.second << ","
	    << padRef[0] << "," << padRef[1] << "," << padRef[2] << "," << padRef[3]
	    << std::endl;

    //NOTE!!!: TH2 bin number is one more then the padNumber
    //Fill pad plane
    if (it.second > cutOff)
      fPadPlane->SetBinContent(it.first+1, it.second);
  }

  fPadPlane->Draw("COL L0");
  fPadPlane->SetMinimum(1.0);
  gStyle->SetOptStat(0);
  gStyle->SetPalette(103);
  gPad ->Update();

}
  

bool cmp(std::pair<Int_t, Int_t> &a,
	 std::pair<Int_t, Int_t> &b)
{
  return a.second > b.second;
}
