{
        TChain *fc = new TChain("tree");
    
        for(int i = 3001; i <= 3094; i++){
		TString ss;
                ss = TString::Format("./simple_%04d.root", i); 
                cout<<ss.Data()<<endl;
                fc->Add(ss.Data());
        }
}
