{
	TChain *fc = new TChain("tree");
	TString ss;
	for(int run_num = 4024; run_num <= 4039; run_num ++){
		if (run_num>=4021 && run_num<=4023) continue;
		ss.Form("../simple_%d.root",run_num);
		cout << ss.Data() << endl;
		fc->Add(ss.Data());
	}

}
