{
//========= Macro generated from object: Li9_tof_e_cut/Graph
//========= by ROOT version6.26/10
   
   cutg = new TCutG("Li9_tof_e_cut",9);
   cutg->SetVarX("rf[2]-ref_tdc");
   cutg->SetVarY("mesh");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetLineColor(2);
   cutg->SetLineWidth(2);
   cutg->SetPoint(0,3933.94,465.998);
   cutg->SetPoint(1,4445.85,486.058);
   cutg->SetPoint(2,4823.17,484.052);
   cutg->SetPoint(3,5295.37,496.088);
   cutg->SetPoint(4,5756.53,488.064);
   cutg->SetPoint(5,5716.81,291.474);
   cutg->SetPoint(6,3964.83,305.517);
   cutg->SetPoint(7,3925.11,463.992);
   cutg->SetPoint(8,3933.94,465.998);
   cutg->Draw("");
}
