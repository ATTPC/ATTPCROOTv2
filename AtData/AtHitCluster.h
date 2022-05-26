#ifndef ATHITCLUSTER_HH
#define ATHITCLUSTER_HH

#include "AtHit.h"

#include <Rtypes.h>
#include <TMatrixDfwd.h>
#include <TMatrixT.h>

class TBuffer;
class TClass;
class TMemberInspector;

class AtHitCluster : public AtHit {
protected:
   TMatrixD fCovMatrix; ///< Cluster covariance matrix

   Double_t fLength = -999;
   Int_t fClusterID = -1;

public:
   AtHitCluster();
   AtHitCluster(const AtHitCluster &cluster) = default;
   virtual ~AtHitCluster() = default;

   void SetCovMatrix(const TMatrixD &matrix) { fCovMatrix = matrix; }
   void SetLength(Double_t length) { fLength = length; }
   void SetClusterID(Int_t id) { fClusterID = id; }

   Double_t GetLength() const { return fLength; }
   const TMatrixD &GetCovMatrix() const { return fCovMatrix; }
   Int_t GetClusterID() const { return fClusterID; }

   ClassDef(AtHitCluster, 3);
};

#endif
