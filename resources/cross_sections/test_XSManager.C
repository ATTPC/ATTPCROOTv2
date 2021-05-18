void test_XSManager()
{
  AtTPCXSManager *xsMan = new AtTPCXSManager();

  gAtXS->SetExcitationFunction("/mnt/simulations/attpcroot/fair_install_2020/ATTPCROOTv2_develop/resources/cross_sections/xs_test.txt");

} 
