{
//========= Macro generated from object: Be11_tof_e_cut/Graph
//========= by ROOT version6.26/10
   
   cutg = new TCutG("Be11_tof_e_cut",25);
   cutg->SetVarX("rf[2]-ref_tdc");
   cutg->SetVarY("mesh");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetLineColor(2);
   cutg->SetLineWidth(2);
   cutg->SetPoint(0,3953.8,504.112);
   cutg->SetPoint(1,4055.3,524.173);
   cutg->SetPoint(2,4068.53,897.292);
   cutg->SetPoint(3,4373.04,851.153);
   cutg->SetPoint(4,4403.93,550.251);
   cutg->SetPoint(5,4397.31,510.13);
   cutg->SetPoint(6,4487.78,542.227);
   cutg->SetPoint(7,4496.6,899.298);
   cutg->SetPoint(8,4816.55,851.153);
   cutg->SetPoint(9,4816.55,520.16);
   cutg->SetPoint(10,4911.43,542.227);
   cutg->SetPoint(11,4915.84,827.081);
   cutg->SetPoint(12,5222.55,827.081);
   cutg->SetPoint(13,5244.62,512.136);
   cutg->SetPoint(14,5332.88,538.215);
   cutg->SetPoint(15,5326.26,903.31);
   cutg->SetPoint(16,5657.24,867.202);
   cutg->SetPoint(17,5688.13,504.112);
   cutg->SetPoint(18,5209.31,476.028);
   cutg->SetPoint(19,4573.83,488.064);
   cutg->SetPoint(20,4207.55,500.1);
   cutg->SetPoint(21,3958.21,470.01);
   cutg->SetPoint(22,3956,502.106);
   cutg->SetPoint(23,3956,502.106);
   cutg->SetPoint(24,3953.8,504.112);
   cutg->Draw("");
}
