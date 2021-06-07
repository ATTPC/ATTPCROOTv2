#include "AtHitCluster.h"

#include "TMath.h"

#include <iostream>

ClassImp(AtHitCluster);

AtHitCluster::AtHitCluster()
{
   Clear();
}

AtHitCluster::AtHitCluster(AtHitCluster *cluster)
{
   fIsClustered = cluster->IsClustered();
   fClusterID = cluster->GetClusterID();
   fTrackID = cluster->GetTrackID();

   fCovMatrix.ResizeTo(3, 3);
   fCovMatrix = cluster->GetCovMatrix();
   fCharge = cluster->GetCharge();
   fLength = cluster->GetLength();

   SetPOCA(cluster->GetPOCA());
   SetPosition(cluster->GetPosition());
   SetTimeStamp(cluster->GetTimeStamp());
   SetPosSigma(cluster->GetPosSigma());
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

void AtHitCluster::SetLength(Double_t length)
{
   fLength = length;
}
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
Double_t AtHitCluster::GetLength()
{
   return fLength;
}
