#ifndef ATHITCLUSTER_HH
#define ATHITCLUSTER_HH

#include <Rtypes.h>
#include <TMatrixDfwd.h>
#include <TMatrixT.h>

#include "AtHit.h"

class TBuffer;
class TClass;
class TMemberInspector;

class AtHitCluster : public AtHit {
protected:
   TMatrixD fCovMatrix; ///< Cluster covariance matrix

   Double_t fLength = -999;
   Double_t fPOCAX = -999;
   Double_t fPOCAY = -999;
   Double_t fPOCAZ = -999;
   Int_t fClusterID = -1;

public:
   AtHitCluster();
   AtHitCluster(const AtHitCluster &cluster) = default;
   virtual ~AtHitCluster() = default;

   void SetCovMatrix(const TMatrixD &matrix) { fCovMatrix = matrix; }
   // void SetPOCA(TVector3 p);
   void SetLength(Double_t length) { fLength = length; }
   void SetClusterID(Int_t id) { fClusterID = id; }

   Double_t GetLength() const { return fLength; }
   const TMatrixD &GetCovMatrix() const { return fCovMatrix; }
   Int_t GetClusterID() const { return fClusterID; }
   // TVector3 GetPOCA();

   ClassDef(AtHitCluster, 2);
};

#endif
