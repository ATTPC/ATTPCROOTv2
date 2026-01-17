{
//========= Macro generated from object: tof_e_cut/Graph
//========= by ROOT version6.26/10
   
   TCutG *cutg = new TCutG("tof_e_cut",19);
   cutg->SetVarX("rf[2]-ref_tdc");
   cutg->SetVarY("mesh");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetLineColor(2);
   cutg->SetLineWidth(2);
   cutg->SetPoint(0,4058.98,494.953);
   cutg->SetPoint(1,4070.2,886.258);
   cutg->SetPoint(2,4169.56,851.32);
   cutg->SetPoint(3,4244.89,525.233);
   cutg->SetPoint(4,4469.25,494.953);
   cutg->SetPoint(5,4483.68,867.624);
   cutg->SetPoint(6,4610.28,851.32);
   cutg->SetPoint(7,4659.96,522.904);
   cutg->SetPoint(8,4890.74,487.966);
   cutg->SetPoint(9,4929.2,867.624);
   cutg->SetPoint(10,5022.16,839.674);
   cutg->SetPoint(11,5065.43,518.245);
   cutg->SetPoint(12,5315.43,506.599);
   cutg->SetPoint(13,5337.87,837.345);
   cutg->SetPoint(14,5442.04,823.37);
   cutg->SetPoint(15,5442.04,487.966);
   cutg->SetPoint(16,4145.52,497.283);
   cutg->SetPoint(17,4052.57,497.283);
   cutg->SetPoint(18,4058.98,494.953);
   cutg->Draw("");
}
