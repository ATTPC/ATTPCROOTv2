void check_entries(){
    TGraph *g1 = new TGraph();
    TGraph *g2 = new TGraph();
    TGraph *g3 = new TGraph();
    int run_begin = 4061;
    int run_end = 4072;

    TString ss;
    int ipoint = 0;

    for(int irun = run_begin; irun <run_end; irun++){
	if (irun == 4067) continue;
        ss.Form("/home/attpc/fair_install/ATTPCROOTv2-RCNPAnalysisBranch/macro/Unpack_HDF5/E581_RCNP/transfer_data/simple_%d_test.root",irun);
        TFile *opf = new TFile(ss.Data(),"READ");
        if(!opf)continue;
        TTree *tree_get = (TTree *)opf->Get("tree");
        if(!tree_get)continue;
        int entries_get = tree_get->GetEntries();
        g1->SetPoint(ipoint, irun, entries_get);

        ss.Form("/data/enData/E581/e581_%d.root",irun);
        TFile *opf1 = new TFile(ss.Data(),"READ");
        if(!opf1)continue;
        TTree *tree_en = (TTree *)opf1->Get("tree");

        int entries_en = tree_en->GetEntries();
        g2->SetPoint(ipoint, irun, entries_en);
        
        g3->SetPoint(ipoint, irun, entries_get - entries_en);
        ipoint++;
        cout<<"irun : "<<irun<<endl;

        
        opf->Close();
        opf1->Close();

    }

    TCanvas *c1 = new TCanvas();
    g1->SetMarkerColor(kRed);
  //  g1->Draw("AP*");
//    g2->Draw("P*same");
    g3->Draw("APL*");
    c1->Draw();
}
