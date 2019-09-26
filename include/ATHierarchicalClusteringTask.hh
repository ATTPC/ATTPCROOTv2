#ifndef ATHIERARCHICALCLUSTERING_H
#define ATHIERARCHICALCLUSTERING_H

#include <vector>

// ATTPCROOT classes
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATDigiPar.hh"
#include "ATHierarchicalClusteringHc.hh"
#include "ATHit.hh"
#include "ATTrajectory.hh"

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"




namespace pcl
{
	namespace visualization
	{
		class PCLVisualizer;
	}
}

class ATHierarchicalClusteringTask : public FairTask
{
public:
	ATHierarchicalClusteringTask();
	~ATHierarchicalClusteringTask();

	virtual InitStatus Init();
	virtual InitStatus ReInit();
	virtual void Exec(Option_t* option);
	virtual void SetParContainers();
	virtual void Finish();

  void SetPersistence(Bool_t value = kTRUE);

	std::vector<ATTrajectory> AnalyzePointArray(std::vector<ATHit> const &hitArray, std::vector<ATHit> *noMatch = nullptr) const;
	//void Visualize(std::vector<ATTrajectory> const &trajectories, std::vector<ATHit> const &noMatch = std::vector<ATHit>()) const;
	//void Visualize(std::vector<ATTrajectory> const &trajectories, std::vector<ATHit> const &noMatch, std::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) const;


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
	ATDigiPar *fPar;

	//ATEvent *fEvent;

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

	ATHierarchicalClusteringTask(const ATHierarchicalClusteringTask&);
	ATHierarchicalClusteringTask operator=(const ATHierarchicalClusteringTask&);

	std::vector<ATTrajectory> useHc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<ATHit> const &hitArray, std::vector<ATHierarchicalClusteringHc::triplet> triplets, float scale, std::vector<ATHit> *noMatch) const;

	ClassDef(ATHierarchicalClusteringTask, 1);
};

#endif
