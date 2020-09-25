//Script for finding fission events in data

void fissionFinder(int runNum = 137)
{
  TChain tpcTree("cbmsim");
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", runNum));

  TTreeReader reader(&tpcTree);
  TTreeReaderValue<TClonesArray> recoArray(reader, "ATRansac");

}

