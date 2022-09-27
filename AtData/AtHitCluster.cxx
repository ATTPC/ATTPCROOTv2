#include "AtHitCluster.h"

#include <FairLogger.h>

#include <Math/Point3D.h>  // for Cartesian3D, operator<<, operator-
#include <Math/Vector3D.h> // for DisplacementVector3D, operator<<, operator*
#include <Rtypes.h>
#include <TError.h>        // for Error
#include <TMatrixTUtils.h> // for TMatrixTRow

#include <array> // for array

ClassImp(AtHitCluster);

constexpr auto scale = 1;

std::unique_ptr<AtHit> AtHitCluster::Clone()
{
   return std::make_unique<AtHitCluster>(*this);
}

AtHitCluster::AtHitCluster() : AtHit(-1, -1, {0, 0, 0}, 0)
{
   fCovMatrix.Zero();
   fCovNumerator.Zero();
   fPositionVariance.SetCoordinates(0, 0, 0);
}

/**
 * @brief Sets the cov[i,j] = cov[j,i] = val.
 */
void AtHitCluster::SetCovMatrix(int i, int j, double val)
{
   fCovMatrix(i, j) = val;
   fCovMatrix(j, i) = val;
}

/**
 * @brief Add hit to cluster.
 *
 * Adds a hit, updating the clusters position and charge. As well as the covariance
 * matrix. The variance is updated using the charge-weighted variance of the passed hit.
 * The off diagonal elements of the covariance matrix are updated according to the "standard"
 * textbook definition of the covariance matrix
 * \f$ COV_{x,y} = \frac{\Sigma_i^N (x_i - \hat{x})(y_i - \hat{y})}{N-1}\f$.
 *
 * The on diagonal elements of the covariance matrix (i.e. the variance) are updated assuming
 * both a charge weighting, and a "reliablility" weighting from the variance in the position of
 * the hits used to construct the hit cluster.
 *
 */
void AtHitCluster::AddHit(const AtHit &hit)
{
   LOG(debug) << "Adding hit " << hit.GetPosition() << " " << hit.GetCharge();
   updateWeightsAndCharge(hit);
   updatePosition(hit);
   updateCovariance(hit);
}

/**
 * Updates the off diagonal covariance numerator according to eqn 15 and the off diagonal
 * variance according to eqn 24.
 * Updates the diagonal elements of the covariance according to eqns 20 and 24
 *
 */
void AtHitCluster::updateCovariance(const AtHit &hit)
{
   // Get array representations of the positions
   std::array<double, 3> hitPos{}, oldPos{}, pos{};

   hit.GetPosition().GetCoordinates(hitPos.begin());
   fPositionChargeOld.GetCoordinates(oldPos.begin());
   fPositionCharge.GetCoordinates(pos.begin());

   // Update off diagonal elements of the covariance matrix
   for (int i = 0; i < 3; ++i)
      for (int j = 0; j < i; ++j) {
         fCovNumerator[i][j] += hit.GetCharge() * (hitPos[i] - pos[i]) * (hitPos[j] - oldPos[j]);
         fCovNumerator[j][i] = fCovNumerator[i][j];
         if (fCharge != 1)
            fCovMatrix[i][j] = fCovNumerator[i][j] / (fCharge - 1);
         else
            fCovMatrix[i][j] = fCovNumerator[i][j] / (fCharge);

         fCovMatrix[j][i] = fCovMatrix[i][j];
      }

   std::array<double, 3> weight{}, totalWeight{}, totalWeight2{};
   fPositionOld.GetCoordinates(oldPos.begin());
   fPosition.GetCoordinates(pos.begin());
   fWeight.GetCoordinates(weight.begin());
   fTotalWeight.GetCoordinates(totalWeight.begin());
   fTotalWeight2.GetCoordinates(totalWeight2.begin());

   // Update on diagonal elements of the covariance matrix
   for (int i = 0; i < 3; ++i) {
      fCovNumerator(i, i) += weight[i] * (hitPos[i] - pos[i]) * (hitPos[i] - oldPos[i]);
      LOG(debug) << "Numerator_" << i << ": " << fCovNumerator(i, i);
      if (fClusterSize == 0)
         fCovMatrix(i, i) = 1 / weight[i] * scale;
      else
         fCovMatrix(i, i) = fCovNumerator(i, i) / (totalWeight[i] - totalWeight2[i] / totalWeight[i]);
   }

   fPositionVariance.SetCoordinates(fCovMatrix(0, 0), fCovMatrix(1, 1), fCovMatrix(2, 2));
}

/**
 * Update the position based on eqn 21.
 *
 * @param[in] hit Hit to update the position with
 */
void AtHitCluster::updatePosition(const AtHit &hit)
{
   fPositionOld = fPosition;
   auto wRatio = ElementDiv(fWeight, fTotalWeight); // Ratio between this weight and the total weight
   fPosition += ElementMult(wRatio, hit.GetPosition() - fPosition);
   LOG(debug) << "Position: " << fPosition;

   fPositionChargeOld = fPositionCharge;
   fPositionCharge += hit.GetCharge() / fCharge * (hit.GetPosition() - fPositionCharge);
   LOG(debug) << "Position charge: " << fPositionCharge;
}

/**
 * @param[in] hit Hit to add to the wights
 * @return weight of hit (1/var *q)
 */
void AtHitCluster::updateWeightsAndCharge(const AtHit &hit)
{

   // Update the weights
   auto wRel = ElementInvert(hit.GetPositionVariance());
   auto wRel2 = ElementMult(wRel, wRel);
   fWeight = hit.GetCharge() * wRel * scale;
   auto w2 = hit.GetCharge() * wRel2 * scale;
   fTotalWeight += fWeight;
   fTotalWeight2 += w2;
   fCharge += hit.GetCharge();
   fClusterSize++;

   LOG(debug) << "w: " << fWeight;
   LOG(debug) << "W: " << fTotalWeight;
   LOG(debug) << "W2: " << fTotalWeight2;
   LOG(debug) << "Q: " << fCharge;
   LOG(debug) << "Denom: " << fTotalWeight - ElementDiv(fTotalWeight2, fTotalWeight);
}

void AtHitCluster::SetPositionVariance(const XYZVector &vec)
{
   SetCovMatrix(0, 0, vec.X());
   SetCovMatrix(1, 1, vec.Y());
   SetCovMatrix(2, 2, vec.Z());
   AtHit::SetPositionVariance(vec);
}
