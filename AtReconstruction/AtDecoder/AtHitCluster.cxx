#include "AtHitCluster.h"

#include "TMath.h"

#include <iostream>

ClassImp(AtHitCluster);

AtHitCluster::AtHitCluster()
{
   Clear();
}

void AtHitCluster::Clear(Option_t *)
{

   fClusterID = -1;

   fCharge = 0;

   fCovMatrix.ResizeTo(3, 3);
   for (Int_t iElem = 0; iElem < 9; iElem++)
      fCovMatrix(iElem / 3, iElem % 3) = 0;

   fLength = -999;

   fPOCAX = -999;
   fPOCAY = -999;
   fPOCAZ = -999;

   // fX = 0;
   // fY = 0;
   // fZ = 0;
}

void AtHitCluster::SetCovMatrix(TMatrixD matrix)
{
   fCovMatrix = matrix;
}
TMatrixD AtHitCluster::GetCovMatrix() const
{
   return fCovMatrix;
}
