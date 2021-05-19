void test_XSManager()
{
   AtTPCXSManager *xsMan = new AtTPCXSManager();

   gAtXS->SetExcitationFunction("/mnt/simulations/attpcroot/fair_install_2020/ATTPCROOTv2_develop/resources/cross_sections/xs_test.txt");
   std::shared_ptr<TH2F> ExFunc = gAtXS->GetExcitationFunction();

   Double_t x = 0;
   Double_t y = 0;
   
   for(auto i=0;i<100;++i){
     ExFunc->GetRandom2(x,y);
     std::cout<<x<<"-"<<y<<"\n";
   }

   ExFunc->Draw("ZCOL");
}
