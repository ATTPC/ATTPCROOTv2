#include "MCSrc.hh"
#include <iostream>
#include "TH1.h"
#include "ATPad.hh"

Int_t main()
{

  #pragma omp parallel for ordered schedule(dynamic,1)
  for(Int_t i=0;i<100;i++)std::cout<<" KUKU "<<std::endl;

  ATPad pad;

  return 0;

}
