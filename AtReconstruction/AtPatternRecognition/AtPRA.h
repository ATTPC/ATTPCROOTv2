#ifndef AtPRA_H
#define AtPRA_H

#include "AtDigiPar.h"
#include "AtTrack.h"
#include "AtHitCluster.h"
#include "AtEvent.h"
#include "AtPatternEvent.h"
#include "AtRansac.h"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

// ROOT
#include "TVirtualFitter.h"

namespace AtPATTERN {

class AtPRA : public TObject {

public:
   virtual ~AtPRA();
   virtual std::vector<AtTrack> GetTrackCand() = 0;
   virtual bool FindTracks(AtEvent &event, AtPatternEvent *patternEvent) = 0;

   void SetTrackInitialParameters(AtTrack &track);
   void PruneTrack(AtTrack &track);
   void SetMaxHits(Int_t maxHits) { fMaxHits = maxHits; }
   void SetMinHits(Int_t minHits) { fMinHits = minHits; }
   void SetMeanDistance(Float_t meanDistance) { fMeanDistance = meanDistance; }
   bool kNN(const std::vector<AtHit> &hits, AtHit &hit, int k);

   void SetkNN(Double_t knn) { fKNN = knn; }
   void SetStdDevMulkNN(Double_t stdDevMul) { fStdDevMulkNN = stdDevMul; }
   void SetkNNDist(Double_t dist) { fkNNDist = dist; }
   void SetPrunning() { kSetPrunning = kTRUE; }

protected:
   FairLogger *fLogger; ///< logger pointer
   AtDigiPar *fPar;     ///< parameter container

   void SetTrackCurvature(AtTrack &track);
   void Clusterize(AtTrack &track);
   void Clusterize3D(AtTrack &track);
   void Clusterize3D(AtTrack &track, Float_t distance, Float_t radius);
   void ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius);

   Int_t fMaxHits;
   Int_t fMinHits;
   Float_t fMeanDistance;
   Int_t fKNN;             //<! Number of nearest neighbors kNN
   Double_t fStdDevMulkNN; //<! Std dev multiplier for kNN
   Double_t fkNNDist;      //<! Distance threshold for outlier rejection in kNN

   Bool_t kSetPrunning; //<<! Enable prunning of tracks

   ClassDef(AtPRA, 1)
};

} // namespace AtPATTERN

Double_t fitf(Double_t *x, Double_t *par);

#endif
