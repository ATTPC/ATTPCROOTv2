#ifndef ATHITCLUSTER_HH
#define ATHITCLUSTER_HH

#include "AtHit.h"

#include "TObject.h"
#include "TVector3.h"
#include "TMatrixD.h"

#include <vector>

class AtHitCluster : public AtHit {
public:
   AtHitCluster();
   AtHitCluster(AtHitCluster *cluster);
   virtual ~AtHitCluster() {}

   void Clear(Option_t * = "");

   void SetCovMatrix(TMatrixD matrix); ///< Set covariance matrix
   TMatrixD GetCovMatrix() const;      ///< Get covariance matrix
   void SetPOCA(TVector3 p);
   TVector3 GetPOCA();
   void SetLength(Double_t length);
   Double_t GetLength();

protected:
   TMatrixD fCovMatrix; ///< Cluster covariance matrix

   Double_t fLength;

   Double_t fPOCAX;
   Double_t fPOCAY;
   Double_t fPOCAZ;

   Int_t fClusterID;

   ClassDef(AtHitCluster, 1);
};

#endif
