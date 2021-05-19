void test_XSManager()
{
   AtTPCXSManager *xsMan = new AtTPCXSManager();

   gAtXS->SetExcitationFunction("/mnt/simulations/attpcroot/fair_install_2020/ATTPCROOTv2_develop/resources/cross_sections/xs_test.txt");
   std::shared_ptr<TH2F> ExFunc = gAtXS->GetExcitationFunction();
   std::shared_ptr<TH2F> ExFunc_test = std::make_shared<TH2F>("ExFunc_test","ExFunc_test",21,0.050,2.150,3,35.0,65.0);
   TH2F *hTest = new TH2F("hTest","hTest",21,0.050,2.150,3,35.0,65.0);
   
   Double_t x = 0;
   Double_t y = 0;
   
   for(auto i=0;i<1000;++i){
     ExFunc->GetRandom2(x,y);
     std::cout<<x<<"-"<<y<<"\n";
     ExFunc_test->Fill(x,y);
     hTest->Fill(x,y);
   }

   TCanvas *c1 = new TCanvas();
   c1->Draw();
   c1->Divide(1,2);
   c1->cd(1);
   ExFunc->Draw("ZCOL");
   c1->cd(2);
   hTest->Draw("ZCOL");
   //ExFunc_test->Draw("ZCOL");
   
}
