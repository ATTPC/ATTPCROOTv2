#ifndef ATHITCLUSTER_HH
#define ATHITCLUSTER_HH

#include "AtHit.h"

#include <Rtypes.h>
#include <TMatrixDSym.h>
#include <TMatrixDfwd.h>
#include <TMatrixT.h>

class TBuffer;
class TClass;
class TMemberInspector;

/**
 * Class representing a cluster of hits that arrise from the same deposition of charge in space.
 * Or at least, that is the assumtion that underlies the math in the class.
 *
 * Will update the position, charge, variance, and covariance as additional hits are added to the
 * cluster. The variance is calcualted using both frequency weights (the charge) and reliability
 * weights (the inverse of the position variance of the hit). The covariance is calcualted just using
 * the frequency weights (charge) w.r.t. the charge-weighted estimator for the true mean (not the
 * frequency and reliability estimator of the mean).
 *
 * Three positions are stored in the cluster:
 * fPositionFull is the position using the full information(charge and hit position variance).
 * fPositionCharge is the position from just charge weighting.
 * fPosition is the position without any weighting applied.
 *
 * There are two covariances accessable:
 * GetCovMatrix() returns the best online covariance matrix described above.
 * GetCovMatrixFull() returns the covariance matrix using fPositionFull and charge
 * weighting. Is a two pass algorithm, no special treatment of the variance.
 * GetCovMatrixCharge() returns a simple charge weighted covariance matrix using fPositionCharge.
 * GetCovMatrixNoWeight() returns a covariance matrix using fPosition and no weighting.
 *
 * Equation numbers in the comments, refer to the document "Hit Clustering" that I need to find
 * somewhere to put it so it is accessable, but it works through the math and assumptions that
 * guide this class.
 *
 * Of course, the covariance matrix can be set
 */
class AtHitCluster : public AtHit {
protected:
   // Off diagonal elements are calcualed using eqn 18. Diagonal from eqn 24
   TMatrixDSym fCovMatrix{3}; //< Cluster covariance matrix

   // Corresponds to equations 15 for off diagonal and 20 for on diagonal
   TMatrixDSym fCovNumerator{3}; //< Numerator for updating covariance matrix

   // The reliability weight for each coordinate is 1/variance.
   XYZVector fTotalWeight{0, 0, 0};  //< Sum of 1/Var*q (Eqn 10)
   XYZVector fTotalWeight2{0, 0, 0}; //< Sum of 1/Var^2*q (Eqn 11)
   XYZVector fPositionOld{0, 0, 0};  //< The last position (used during update)
   XYZVector fWeight{0, 0, 0};       //< The weight of the point being added (used during update)

   XYZPoint fPositionUnweighted{0, 0, 0};
   XYZPoint fPositionUnweightedOld{0, 0, 0};

   XYZPoint fPositionCharge{0, 0, 0};
   XYZPoint fPositionChargeOld{0, 0, 0};

   Double_t fLength{-999};
   Int_t fClusterID{-1};
   Int_t fClusterSize{0}; //< Number of hits in the cluster

   std::vector<AtHit> fHits;

public:
   AtHitCluster();
   AtHitCluster(const AtHitCluster &cluster) = default;
   virtual ~AtHitCluster() = default;
   virtual std::unique_ptr<AtHit> Clone() override; //< Create a copy of sub-type

   void SetCovMatrix(TMatrixDSym matrix) { fCovMatrix = std::move(matrix); }
   void SetCovMatrix(int i, int j, double val);
   virtual void SetPositionVariance(const XYZPoint &vec) override;
   void SetLength(Double_t length) { fLength = length; }
   void SetClusterID(Int_t id) { fClusterID = id; }
   void AddHit(const AtHit &hit);

   Double_t GetLength() const { return fLength; }
   XYZPoint GetPositionCharge() const { return fPositionCharge; }
   XYZPoint GetPositionUnWeighted() const { return fPositionUnweighted; }

   const TMatrixDSym &GetCovMatrix() const { return fCovMatrix; }
   TMatrixDSym GetCovMatrixFull() const;
   TMatrixDSym GetCovMatrixCharge() const;
   TMatrixDSym GetCovMatrixNoWeight() const;

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
