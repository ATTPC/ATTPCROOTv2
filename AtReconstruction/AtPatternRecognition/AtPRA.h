#ifndef AtPRA_H
#define AtPRA_H

#include <Rtypes.h>  // for Double_t, Float_t, Int_t, THashConsistencyHolder
#include <TObject.h> // for TObject

#include <memory>
#include <vector> // for vector

class AtDigiPar;
class AtEvent;
class AtHit;
class AtPatternEvent;
class AtTrack;
class TBuffer;
class TClass;
class TMemberInspector;

namespace AtPATTERN {

class AtPRA : public TObject {
protected:
   AtDigiPar *fPar; ///< parameter container

   Int_t fMaxHits{5000};
   Int_t fMinHits{0};
   Float_t fMeanDistance{1e9};
   Int_t fKNN{5};             //<! Number of nearest neighbors kNN
   Double_t fStdDevMulkNN{0}; //<! Std dev multiplier for kNN
   Double_t fkNNDist{10};     //<! Distance threshold for outlier rejection in kNN

   Bool_t kSetPrunning{false}; //<<! Enable prunning of tracks

   Bool_t fCirclePCL{false};
   Bool_t fAnglePCL{false};

public:
   virtual ~AtPRA() = default;
   virtual std::vector<AtTrack> GetTrackCand() = 0;
   virtual std::unique_ptr<AtPatternEvent> FindTracks(AtEvent &event) = 0;

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

   void SetCirclePCL(bool val) { fCirclePCL = val; }
   void SetAnglePCL(bool val) { fAnglePCL = val; }

protected:
   void SetTrackCurvature(AtTrack &track);
   void Clusterize(AtTrack &track);
   void Clusterize3D(AtTrack &track);
   void Clusterize3D(AtTrack &track, Float_t distance, Float_t radius);
   void ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius);

   ClassDef(AtPRA, 1)
};

} // namespace AtPATTERN

Double_t fitf(Double_t *x, Double_t *par);

#endif
