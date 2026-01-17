{
//========= Macro generated from object: He_cut_small/Graph
//========= by ROOT version6.26/10
   
   TCutG *cutg = new TCutG("He_cut_small",10);
   cutg->SetVarX(" range[0]");
   cutg->SetVarY("dedx[0] ");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetPoint(0,39.3964,1304.82);
   cutg->SetPoint(1,82.2186,861.659);
   cutg->SetPoint(2,179.486,610.38);
   cutg->SetPoint(3,250.449,468.75);
   cutg->SetPoint(4,250.449,697.186);
   cutg->SetPoint(5,117.7,1126.64);
   cutg->SetPoint(6,52.2431,1597.22);
   cutg->SetPoint(7,28.9967,1391.63);
   cutg->SetPoint(8,39.3964,1295.69);
   cutg->SetPoint(9,39.3964,1304.82);
   cutg->Draw("");
}
