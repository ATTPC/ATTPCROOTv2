#include "AtHitClusterFull.h"

#include <Math/Point3D.h>  // for PositionVector3D
#include <TError.h>        // for Error
#include <TMatrixTSym.h>   // for TMatrixTSym
#include <TMatrixTUtils.h> // for TMatrixTRow

#include <algorithm> // for max
#include <array>     // for array

void AtHitClusterFull::AddHit(const AtHit &hit)
{
   AtHitCluster::AddHit(hit);
   fHits.push_back(hit);
}

AtHit::XYZPoint AtHitClusterFull::GetPositionUnWeighted() const
{
   XYZPoint pos{0, 0, 0};
   for (auto &hit : fHits) {
      auto hitPos = hit.GetPosition();
      pos.SetCoordinates(pos.X() + hitPos.X(), pos.Y() + hitPos.Y(), pos.Z() + hitPos.Z());
   }
   return pos / fHits.size();
}

TMatrixDSym AtHitClusterFull::GetCovMatrixNoWeight() const
{
   TMatrixDSym cov{3};
   cov.Zero();

   std::array<double, 3> pos{}, hitPos{};
   GetPositionUnWeighted().GetCoordinates(pos.begin());
   for (const auto &hit : fHits) {
      hit.GetPosition().GetCoordinates(hitPos.begin());

      for (int i = 0; i < 3; ++i)
         for (int j = 0; j < 3; ++j)
            cov[i][j] += (hitPos[i] - pos[i]) * (hitPos[j] - pos[j]) / (fClusterSize - 1);
   }

   return cov;
}

TMatrixDSym AtHitClusterFull::GetCovMatrixCharge() const
{
   TMatrixDSym cov{3};
   cov.Zero();

   std::array<double, 3> pos{}, hitPos{};
   fPositionCharge.GetCoordinates(pos.begin());
   for (const auto &hit : fHits) {
      hit.GetPosition().GetCoordinates(hitPos.begin());

      for (int i = 0; i < 3; ++i)
         for (int j = 0; j < 3; ++j)
            cov[i][j] += hit.GetCharge() * (hitPos[i] - pos[i]) * (hitPos[j] - pos[j]) / (fCharge - 1);
   }

   return cov;
}

TMatrixDSym AtHitClusterFull::GetCovMatrixFull() const
{
   TMatrixDSym cov{3};
   cov.Zero();

   std::array<double, 3> pos{}, hitPos{};
   fPosition.GetCoordinates(pos.begin());
   for (const auto &hit : fHits) {
      hit.GetPosition().GetCoordinates(hitPos.begin());

      for (int i = 0; i < 3; ++i)
         for (int j = 0; j < 3; ++j)
            cov[i][j] += hit.GetCharge() * (hitPos[i] - pos[i]) * (hitPos[j] - pos[j]) / (fCharge - 1);
   }

   return cov;
}
