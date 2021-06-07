// Macro to look for runs missing the TS information
// NSCL: 310 TPC: 118
// NSCL: 407 TPC: 200


void getMissingTS(int minRun, int maxRun)
{
  for(int i = minRun; i <= maxRun; ++i)
  {
    //Open the tree
    TChain trTpc("cbmsim");
    trTpc.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", i));
    TTreeReader tpcTr(&trTpc);
    TTreeReaderValue<TClonesArray> tpcArray(tpcTr, "ATRawEvent");

    //Get the timestamp value
    if( !tpcTr.Next())
    {
      std::cout << "Error reading info from run " << i << std::endl;
      continue;
    }

    auto event = (ATRawEvent*) tpcArray->At(0);
    auto tpcTS = event->GetTimestamp();

    if (tpcTS == 0)
      std::cout << "Missing timestamp info from run " << i << std::endl;
  }
}
