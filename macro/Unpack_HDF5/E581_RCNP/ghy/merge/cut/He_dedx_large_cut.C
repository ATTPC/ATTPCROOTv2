{
//========= Macro generated from object: He_dedx_large_cut/Graph
//========= by ROOT version6.26/10
   
   cutg = new TCutG("He_dedx_large_cut",8);
   cutg->SetVarX("range[0]");
   cutg->SetVarY("dedx[0]");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetLineColor(2);
   cutg->SetLineWidth(2);
   cutg->SetPoint(0,12.7427,1780.22);
   cutg->SetPoint(1,370.201,516.424);
   cutg->SetPoint(2,1068.57,309.554);
   cutg->SetPoint(3,1066.91,828.611);
   cutg->SetPoint(4,183.197,2886.03);
   cutg->SetPoint(5,34.2564,2528.71);
   cutg->SetPoint(6,22.6721,1735.08);
   cutg->SetPoint(7,12.7427,1780.22);
   cutg->Draw("");
}
