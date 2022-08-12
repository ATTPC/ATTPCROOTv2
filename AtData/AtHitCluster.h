#ifndef ATHITCLUSTER_HH
#define ATHITCLUSTER_HH

#include "AtHit.h"

#include <Rtypes.h>
#include <TMatrixDSymfwd.h> // for TMatrixDSym
#include <TMatrixTSym.h>    // for TMatrixTSym

#include <memory>  // for unique_ptr
#include <utility> // for move

class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief: Class representing a cluster of hits that arise from the same deposition of charge in space.
 * Or at least, that is the assumtion that underlies the math in the class.
 *
 * Will update the position, charge, variance, and covariance as additional hits are added to the
 * cluster. The variance is calcualted using both frequency weights (the charge) and reliability
 * weights (the inverse of the position variance of the hit). The covariance is calculated just using
 * the frequency weights (charge) w.r.t. the charge-weighted estimator for the true mean (not the
 * frequency and reliability estimator of the mean). Eventually, I will sit down and work out a
 * recursive expression for the covariance using the best estimate of the mean instead of just their
 * charge weighted.
 *
 * The covariance and matrix can also be manually set like the old version of the class.
 *
 * Three positions are stored in the cluster:
 * fPositionFull is the position using the full information(charge and hit position variance).
 * fPositionCharge is the position from just charge weighting.
 * fPosition is the position without any weighting applied.
 *
 */
class AtHitCluster : public AtHit {
protected:
   Int_t fClusterID{-1};

   // Off diagonal elements are calcualed using eqn 18. Diagonal from eqn 24
   TMatrixDSym fCovMatrix{3}; //< Cluster covariance matrix

   // Corresponds to equations 15 for off diagonal and 20 for on diagonal
   TMatrixDSym fCovNumerator{3}; //< Numerator for updating covariance matrix

   // The reliability weight for each coordinate is 1/variance.
   XYZVector fTotalWeight{0, 0, 0};  //< Sum of 1/Var*q (Eqn 10)
   XYZVector fTotalWeight2{0, 0, 0}; //< Sum of 1/Var^2*q (Eqn 11)
   XYZVector fWeight{0, 0, 0};       //< The weight of the point being added (used during update)

   XYZPoint fPositionCharge{0, 0, 0};    //< Charge weighted mean of position
   XYZPoint fPositionChargeOld{0, 0, 0}; //< The last charge weighted position
   XYZVector fPositionOld{0, 0, 0};      //< The last position (used during update)

   Double_t fLength{-999};
   Int_t fClusterSize{0}; //< Number of hits in the cluster

public:
   AtHitCluster();
   AtHitCluster(const AtHitCluster &cluster) = default;
   virtual ~AtHitCluster() = default;
   virtual std::unique_ptr<AtHit> Clone() override; //< Create a copy of sub-type

   void SetCovMatrix(TMatrixDSym matrix) { fCovMatrix = std::move(matrix); }
   void SetCovMatrix(int i, int j, double val);
   virtual void SetPositionVariance(const XYZVector &vec) override;
   void SetLength(Double_t length) { fLength = length; }
   void SetClusterID(Int_t id) { fClusterID = id; }
   virtual void AddHit(const AtHit &hit);

   Double_t GetLength() const { return fLength; }
   XYZPoint GetPositionCharge() const { return fPositionCharge; }

   const TMatrixDSym &GetCovMatrix() const { return fCovMatrix; }
   const TMatrixDSym &GetCovNumerator() const { return fCovNumerator; }

   Int_t GetClusterID() const { return fClusterID; }
   Int_t GetClusterSize() const { return fClusterSize; }

protected:
   template <class A, class B>
   XYZVector ElementMult(const A &a, const B &b)
   {
      return {a.X() * b.X(), a.Y() * b.Y(), a.Z() * b.Z()};
   }
   template <class A, class B>
   XYZVector ElementDiv(const A &a, const B &b)
   {
      return {a.X() / b.X(), a.Y() / b.Y(), a.Z() / b.Z()};
   }
   template <class T>
   T ElementInvert(const T &b)
   {
      return {1 / b.X(), 1 / b.Y(), 1 / b.Z()};
   }

   void updateWeightsAndCharge(const AtHit &hit);
   void updatePosition(const AtHit &hit);
   void updateCovariance(const AtHit &hit);

   ClassDefOverride(AtHitCluster, 4);
};

#endif
