{
	TChain *fc = new TChain("tree");
	TChain *fc_gagg = new TChain("tree");
	TString ss;
	for(int run_num = 4063; run_num <= 4066; run_num ++){

		if (run_num >=4046 && run_num<= 4050) continue;
		
		ss.Form("../simple_%d_test.root",run_num);
		cout << ss.Data() << endl;
		fc->Add(ss.Data());
		
		ss = TString::Format("/data/sustech/user/ghy/frib-decode/data/gagg_%04d.root", run_num);
                cout<<ss.Data()<<endl;
                fc_gagg->Add(ss.Data());
	}
	fc->AddFriend(fc_gagg)

}
