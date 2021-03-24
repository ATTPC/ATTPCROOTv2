#include "ATHitCluster.hh"

#include "TMath.h"

#include <iostream>

ClassImp(ATHitCluster);

ATHitCluster::ATHitCluster()
{
  Clear();
}

void ATHitCluster::Clear(Option_t *)
{

  fClusterID = -1;

  fCharge = 0;

  fCovMatrix.ResizeTo(3, 3);
  for (Int_t iElem = 0; iElem < 9; iElem++)
    fCovMatrix(iElem/3, iElem%3) = 0;


  fLength = -999;

  fPOCAX = -999;
  fPOCAY = -999;
  fPOCAZ = -999;

  //fX = 0;
  //fY = 0;
  //fZ = 0;

 
}

void     ATHitCluster::SetCovMatrix(TMatrixD matrix) { fCovMatrix = matrix; }
TMatrixD ATHitCluster::GetCovMatrix()         const  { return fCovMatrix; }

