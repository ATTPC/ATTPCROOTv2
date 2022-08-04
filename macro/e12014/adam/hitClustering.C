#include "helper.h"

void hitClustering()
{
   fair::Logger::SetConsoleSeverity("debug");
   // padnum, {loc}, charge
   AtHit hit[3];
   hit[0].SetCharge(2);
   hit[1].SetCharge(1);
   hit[2].SetCharge(2);

   hit[0].SetPosition({5, 5, 5});
   hit[1].SetPosition({9, 9, 9});
   hit[2].SetPosition({6, 6, 6});

   hit[0].SetPositionVariance({3 * 3, 16, 1});
   hit[1].SetPositionVariance({5 * 5, 1, 1});
   hit[2].SetPositionVariance({2 * 2, 16, 1});

   AtHitClusterFull cluster;
   cluster.AddHit(hit[0]);
   cluster.GetCovNumerator().Print();
   cluster.GetCovMatrix().Print();
   cluster.AddHit(hit[1]);
   cluster.GetCovNumerator().Print();
   cluster.GetCovMatrix().Print();
   cluster.AddHit(hit[2]);

   std::cout << "Online estimate" << std::endl;
   cluster.GetCovMatrix().Print();

   std::cout << "No Weighting" << std::endl;
   cluster.GetCovMatrixNoWeight().Print();

   std::cout << "Charge Weighting" << std::endl;
   cluster.GetCovMatrixCharge().Print();

   std::cout << "Charge Weighting with best postion" << std::endl;
   cluster.GetCovMatrixFull().Print();
}
