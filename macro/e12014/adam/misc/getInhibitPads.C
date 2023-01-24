#include "helper.h"

#include "TH2Poly.h"
#include <fstream>

void getInhibitPads(double radius)
{
   auto tpcMap = new AtTpcMap();
   tpcMap->ParseXMLMap(TString(gSystem->Getenv("VMCWORKDIR")) + "/scripts/e12014_pad_map_size.xml");
   tpcMap->GenerateAtTpc();
   std::ofstream outFile("gainReduction.txt");
   if(!outFile)
   {
      std::cout << "Failed to open file to write!" << std::endl;
      return;
      
   }
   auto padPlane = tpcMap->GetAtTpcPlane();

   for(int i = 0; i < padPlane->GetNumberOfBins(); ++i)
   {
      auto pad = tpcMap->BinToPad(i);
      auto center = tpcMap->CalcPadCenter(pad);

      auto rho = TMath::Sqrt(center[0]*center[0] + center[1]*center[1]);
      if(rho < radius)
      {
	 padPlane->Fill(center[0], center[1]);
	 outFile << pad << std::endl;
      }
	 
   }
   padPlane->Draw();
}
