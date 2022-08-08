#ifndef ATHITCLUSTERFULL_HH
#define ATHITCLUSTERFULL_HH

#include "AtHit.h" // for AtHit, AtHit::XYZPoint
#include "AtHitCluster.h"

#include <Rtypes.h>         // for THashConsistencyHolder, ClassDefOverride
#include <TMatrixDSymfwd.h> // for TMatrixDSym

#include <memory> // for unique_ptr
#include <vector> // for vector
class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief: Class representing a cluster of hits that arise from the same deposition of charge in space.
 * Or at least, that is the assumtion that underlies the math in the class.
 *
 * In addition to the information tracked by AtHitCluster, it also saves a copy of every hit that
 * is part of the cluster. This allows for a larger number of estimates of the variance and covariance
 * to be calcualted.
 *
 *
 * Three estimates of the mean are stored in the cluster:
 * fPosition is the position using the full information(charge and hit position variance).
 * fPositionCharge is the position from just charge weighting.
 * fPositionUnweighted is the position without any weighting applied.
 *
 * There are four estimates of the covariances accessable:
 * GetCovMatrix() returns the best online covariance matrix described above (same as AtHitCluster).
 * GetCovMatrixFull() returns the covariance matrix using fPositionFull and charge weighting. There
 * is no special treatment of the variance.
 * GetCovMatrixCharge() returns a simple charge weighted covariance matrix using fPositionCharge.
 * GetCovMatrixNoWeight() returns a covariance matrix using fPosition and no weighting.
 *
 */
class AtHitClusterFull : public AtHitCluster {
protected:
   std::vector<AtHit> fHits;

public:
   AtHitClusterFull() : AtHitCluster() {}
   AtHitClusterFull(const AtHitClusterFull &cluster) = default;
   virtual ~AtHitClusterFull() = default;
   virtual std::unique_ptr<AtHit> Clone() override; //< Create a copy of sub-type

   virtual void AddHit(const AtHit &hit) override;

   XYZPoint GetPositionUnWeighted() const;
   TMatrixDSym GetCovMatrixFull() const;
   TMatrixDSym GetCovMatrixCharge() const;
   TMatrixDSym GetCovMatrixNoWeight() const;

   ClassDefOverride(AtHitClusterFull, 1);
};

#endif //#ifndef ATHITCLUSTERFULL_HH
