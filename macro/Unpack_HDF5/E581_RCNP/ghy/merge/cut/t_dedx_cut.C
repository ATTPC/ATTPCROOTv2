{
//========= Macro generated from object: t_dedx_cut/Graph
//========= by ROOT version6.26/10
   
   TCutG *cutg = new TCutG("t_dedx_cut",12);
   cutg->SetVarX("range[0]");
   cutg->SetVarY("dedx[0]");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetLineColor(2);
   cutg->SetLineWidth(2);
   cutg->SetPoint(0,23.6484,1414.52);
   cutg->SetPoint(1,56.9753,1104.84);
   cutg->SetPoint(2,98.5198,879.032);
   cutg->SetPoint(3,140.521,737.097);
   cutg->SetPoint(4,193.935,546.774);
   cutg->SetPoint(5,248.263,424.194);
   cutg->SetPoint(6,275.198,417.742);
   cutg->SetPoint(7,279.763,514.516);
   cutg->SetPoint(8,167.456,959.677);
   cutg->SetPoint(9,28.2137,1617.74);
   cutg->SetPoint(10,23.1919,1414.52);
   cutg->SetPoint(11,23.6484,1414.52);
   cutg->Draw("");
}
