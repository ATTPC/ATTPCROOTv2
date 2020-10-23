{
TFile *f = new TFile("cuts_jp.root");
Ni69;
Ni68;
Ni67;
Ni66;

TChain *ch = new TChain("anatree");
/*ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2020_0020.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2021_0021_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2022_0022_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2023_0023_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2024_0024_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2025_0025_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2026_0026_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2027_0027_jp.root");
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2028_0028_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2029_0029_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2030_0030_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2031_0031.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2032_0032.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2033_0033.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2034_0034.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2035_0035.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2036_0036.root"); */
// keep adding files here
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2077_0077_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2079_0079_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2085_0085_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2086_0086_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2087_0087_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2088_0088_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2101_0101_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2103_0103_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2104_0104_jp.root");
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2105_0105_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2108_0108_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2109_0109_jp.root");  
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2110_0110_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2111_0111_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2113_0113_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2114_0114_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2115_0115_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2116_0116_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2117_0117_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2120_0120_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2124_0124_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2125_0125_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2126_0126_jp.root");
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2127_0127_jp.root");
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2128_0128_jp.root");
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2129_0129_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2130_0130_jp.root");    
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2137_0137_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2138_0138_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2139_0139_jp.root"); 
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2141_0141_jp.root");
ch->AddFile("/mnt/analysis/e18027/rootAna/runAnalyzed_2142_0142_jp.root"); 



TCanvas *c = new TCanvas(); 
c->Divide(2,3);
c->cd(1);
ch->Draw("theta1*TMath::RadToDeg()","","");
ch->Draw("S800_XfObj_tof:S800_ObjCorr>>hj(100,-100,20,100,220,300)","","");
//ch->Draw("eLoss_p1_reco:theta_cm>>kine(50,0,5,50,0,5)","MaxR1<230.0","colz");
Ni69->Draw("same");
Ni68->Draw("same");
Ni67->Draw("same");
Ni66->Draw("same");
c->cd(2);
//ch->Draw("S800_ICSum:S800_ObjCorr","","");
//cuts1->Draw("same");
ch->Draw("Ex4>>h1(100,-50,50","theta1>7.0*3.41/180.0 && theta1<170*3.14/180.0 && vertexZ>70 && theta_cm>1.0 && theta_cm<2.0 ","");
//ch->Draw("Ex4>>h(50,-10,40","theta1>60.0*3.14/180.0 ","");
c->cd(3);
//ch->Draw("Ex4>>h(45,-50,-5","theta1>7.0*3.14/180.0 && theta1<170*3.14/180.0 && vertexZ>50 ","");
//ch->Draw("eLoss_p1_reco:theta1*TMath::RadToDeg()","vertexZ>50.0 && MaxR1<230.0","colz");
ch->Draw("Ex4>>h2(100,-50,50","theta1>7.0*3.41/180.0 && theta1<90*3.14/180.0 && vertexZ>70 && Ni69 && theta_cm>1.0 && theta_cm<2.0","");
c->cd(4);
//ch->Draw("Ex4>>h11(120,-50,50","theta1>7.0*3.41/180.0 && theta1<170*3.14/180.0 && vertexZ>50","");
ch->Draw("Ex4>>h3(100,-50,50","theta1>7.0*3.41/180.0 && theta1<90*3.14/180.0 && vertexZ>70 && Ni68 && theta_cm>1.0 && theta_cm<2.0","");
c->cd(5);
//ch->Draw("vertexZ:theta1*TMath::RadToDeg()","","");
ch->Draw("Ex4>>h4(50,-50,50","theta1>7.0*3.41/180.0 && theta1<90*3.14/180.0 && vertexZ>70 && Ni67 && theta_cm>1.0 && theta_cm<2.0","");
c->cd(6);
//ch->Draw("vertexZ:theta1*TMath::RadToDeg()","","");

ch->Draw("Ex4>>h5(100,-50,50","theta1>7.0*3.41/180.0 && theta1<90*3.14/180.0 && vertexZ>70 && Ni66 && theta_cm>1.0 && theta_cm<2.0","");

//ch->Draw("Ex4>>hh(100,-50,50","theta1>7.0*3.14/180.0 && theta1<170*3.14/180.0 && vertexZ>50 && eLoss_p1_reco<1.5 && theta_cm>0.5","");
//ch->Draw("Ex4>>h12(350,-10,40","theta1>20.0*3.14/180.0 && theta1<170*3.14/180.0 && vertexZ>0 && MaxR1<240.0","");


}
