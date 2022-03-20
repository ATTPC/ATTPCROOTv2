#include "AtHitCluster.h"

#include "TMath.h"

#include <iostream>

ClassImp(AtHitCluster);

AtHitCluster::AtHitCluster() : AtHit(-1, 0, 0, -1000, 0)
{
   fCovMatrix.ResizeTo(3, 3);
   for (Int_t iElem = 0; iElem < 9; iElem++)
      fCovMatrix(iElem / 3, iElem % 3) = 0;
}
/*
void AtHitCluster::SetPOCA(TVector3 p)
{
   fPOCAX = p.X();
   fPOCAY = p.Y();
   fPOCAZ = p.Z();
}

TVector3 AtHitCluster::GetPOCA()
{
   return TVector3(fPOCAX, fPOCAY, fPOCAZ);
}
*/
