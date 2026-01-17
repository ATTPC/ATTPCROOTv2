void check_run(int run_num){
    gROOT->Macro("/home/attpc/fair_install/ATTPCROOTv2-RCNPAnalysisBranch/macro/Unpack_HDF5/E581_RCNP/ghy/cut/d_cut_newpar.C");


    TString ss;
    TCanvas *c1 = new TCanvas("c0");
    c1->Divide(2, 2);
    //check unpack output:
    ss.Form("/home/attpc/fair_install/ATTPCROOTv2-RCNPAnalysisBranch/macro/Unpack_HDF5/E581_RCNP/transfer_data/simple_%d_test.root", run_num);
    TFile *opf = new TFile(ss.Data(), "READ");
    if(!opf)cout<<"fail to read unpacker output file : "<<run_num<<endl;
    TTree *tree = (TTree *)opf->Get("tree");
    
    ss.Form("/data/sustech/user/ghy/frib-decode/data/gagg_%d.root", run_num);
    TFile *opf_gagg = new TFile(ss.Data(), "READ");
    if(!opf_gagg)cout<<"fail to read gagg output file : "<<run_num<<endl;
    TTree *tree_gagg = (TTree *)opf_gagg->Get("tree");

    tree->AddFriend(tree_gagg);

    c1->cd(1);
    gPad->SetGridx();
    gPad->SetGridy();
    tree->Draw("dedx[0] : range[0]>>h1(500, 0, 600, 500, 2000)","punch_through[0] == 0","colz");
    c1->cd(2);
    gPad->SetGridx();
    gPad->SetGridy();
    tree->Draw("SiFe1 : SiFe2>>h2(500, 0, 4000, 500, 0, 2000)","punch_through[0] == 0 && SiFn1 == 1 && SiFn2 == 1","colz");
    c1->cd(3);
    gPad->SetGridx();
    gPad->SetGridy();
    tree->Draw("SiFe2 : e_g1[0]>>h3(500, 0, 6000, 500, 0, 4000)","punch_through[0] == 0 && SiFn1 == 1 && SiFn2 == 1 &&id_g1[0] < 16","colz");
    c1->cd(4);
    gPad->SetGridx();
    gPad->SetGridy();
    tree->Draw("kineE_d[0] : thetaLab[0]>>h4(100, 0, 100, 100, 0, 15)","punch_through[0] == 0 && SiFn1 == 1 && SiFn2 == 1 && track_num == 1 && d_cut_newpar","");

    c1->Draw();

}