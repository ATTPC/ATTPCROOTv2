#ifndef AtHIERARCHICALCLUSTERING_H
#define AtHIERARCHICALCLUSTERING_H

#include <vector>

// AtTPCROOT classes
#include "AtEvent.h"
#include "AtProtoEvent.h"
#include "AtDigiPar.h"
#include "AtHierarchicalClusteringHc.h"
#include "AtHit.h"
#include "AtTrajectory.h"

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

namespace pcl {
namespace visualization {
class PCLVisualizer;
}
} // namespace pcl

class AtHierarchicalClusteringTask : public FairTask {
public:
   AtHierarchicalClusteringTask();
   ~AtHierarchicalClusteringTask();

   virtual InitStatus Init();
   virtual InitStatus ReInit();
   virtual void Exec(Option_t *option);
   virtual void SetParContainers();
   virtual void Finish();

   void SetPersistence(Bool_t value = kTRUE);

   std::vector<AtTrajectory>
   AnalyzePointArray(std::vector<AtHit> const &hitArray, std::vector<AtHit> *noMatch = nullptr) const;
   // void Visualize(std::vector<AtTrajectory> const &trajectories, std::vector<AtHit> const &noMatch =
   // std::vector<AtHit>()) const; void Visualize(std::vector<AtTrajectory> const &trajectories, std::vector<AtHit>
   // const &noMatch, std::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) const;

   // Getters and Setters
   void SetBestClusterDistanceDelta(float value);
   float GetBestClusterDistanceDelta() const;

   void SetCleanupMinTriplets(size_t value);
   size_t GetCleanupMinTriplets() const;

   void SetCloudScaleModifier(float value);
   float GetCloudScaleModifier() const;

   void SetGenTripletsMaxError(float value);
   float GetGenTripletsMaxError() const;

   void SetGenTripletsNnKandidates(size_t value);
   size_t GetGenTripletsNnKandidates() const;

   void SetGenTripletsNBest(size_t value);
   size_t GetGenTripletsNBest() const;

   void SetSmoothRadius(float value);
   float GetSmoothRadius() const;

   void SetSplineTangentScale(float value);
   float GetSplineTangentScale() const;

   void SetSplineMinControlPointDistance(float value);
   float GetSplineMinControlPointDistance() const;

   void SetSplineJump(size_t value);
   size_t GetSplineJump() const;

private:
   TClonesArray *fEventHArray;
   TClonesArray *fHierarchicalClusteringArray;

   FairLogger *fLogger;
   AtDigiPar *fPar;

   // AtEvent *fEvent;

   Bool_t kIsPersistence;

   float _bestClusterDistanceDelta = 2.0f;
   size_t _cleanupMinTriplets = 4;
   float _cloudScaleModifier = 4.0f;
   float _genTripletsMaxError = 0.01f;
   size_t _genTripletsNnKandidates = 10;
   size_t _genTripletsNBest = 2;
   float _smoothRadius = 5.0f;
   float _splineTangentScale = 0.5f;
   float _splineMinControlPointDistance = 20.0f;
   size_t _splineJump = 1;

   AtHierarchicalClusteringTask(const AtHierarchicalClusteringTask &);
   AtHierarchicalClusteringTask operator=(const AtHierarchicalClusteringTask &);

   std::vector<AtTrajectory> useHc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<AtHit> const &hitArray,
                                   std::vector<AtHierarchicalClusteringHc::triplet> triplets, float scale,
                                   std::vector<AtHit> *noMatch) const;

   ClassDef(AtHierarchicalClusteringTask, 1);
};

#endif
