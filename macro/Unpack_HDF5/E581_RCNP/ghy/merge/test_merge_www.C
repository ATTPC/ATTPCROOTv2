void test_merge_www(){
    TString ss;
	
    for (int irun = 4045; irun<= 4045; irun++)
    {
    if (irun == 4067) continue;
    ss.Form("/home/attpc/fair_install/ATTPCROOTv2-RCNPAnalysisBranch/macro/Unpack_HDF5/E581_RCNP/transfer_data/simple_%d_test.root",irun);
    TFile *opf = new TFile(ss.Data(),"READ");
    cout << ss.Data() << endl;
    TTree *tree_get = (TTree *)opf->Get("tree");
//    if(!tree_get)continue;
    int entries_get = tree_get->GetEntries();

    ss.Form("/data/enData/E581/e581_%d.root",irun);
    TFile *opf1 = new TFile(ss.Data(),"READ");
    cout << ss.Data() << endl;
  //  if(!opf1)continue;
    TTree *tree_en = (TTree *)opf1->Get("tree");
    int entries_en = tree_en->GetEntries();

    ss.Form("/data/sustech/user/ghy/frib-decode/data/gagg_%d.root", irun);
    TFile *opf_gagg = new TFile(ss.Data(), "READ");
    cout << ss.Data() << endl;
    if(!opf_gagg)cout<<"fail to read gagg output file : "<<irun<<endl;
    TTree *tree_gagg = (TTree *)opf_gagg->Get("tree");
 
    ss.Form("./merge_data/merge_%d_www.root", irun);
    
    TFile *ipf = new TFile(ss.Data(),"RECREATE");
    ipf->cd();
    int total_entries = entries_en;
    TTree *tree_get_new = tree_get->CloneTree(total_entries);
    TTree *tree_gagg_new = tree_gagg->CloneTree(total_entries);
    TTree *tree_en_new = tree_en->CloneTree(total_entries);
    //tree_get_new->Write();
    //tree_gagg_new->Write();
    tree_get_new->Write("tree_get");
    tree_gagg_new->Write("tree_frib");
    tree_en_new->Write("tree_en");

    ipf->Close();
    }
}
