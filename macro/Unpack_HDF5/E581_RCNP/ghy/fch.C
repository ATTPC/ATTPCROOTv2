{
	TChain *fc = new TChain("tree");
	TString ss;
	for(int run_num = 4024; run_num <= 4070; run_num ++){
		ss.Form("../transfer_data/simple_%d_test.root",run_num);
		fc->Add(ss.Data());
	}

}
