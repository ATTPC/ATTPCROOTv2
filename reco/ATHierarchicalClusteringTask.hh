#ifndef ATHIERARCHICALCLUSTERING_H
#define ATHIERARCHICALCLUSTERING_H

#include <vector>

#include "FairTask.h"

#include "ATHierarchicalClusteringHc.hh"
#include "ATHit.hh"
#include "ATTrajectory.hh"

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

	std::vector<ATTrajectory> AnalyzePointArray(std::vector<ATHit> const &hitArray) const;


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

private:
	/** Input array from previous already existing data level **/
	//  TClonesArray* <InputDataLevel>;

	/** Output array to new data level**/
	//  TClonesArray* <OutputDataLevel>;

	float _bestClusterDistanceDelta = 2.91713f;
	size_t _cleanupMinTriplets = 20;
	float _cloudScaleModifier = 0.281718f;
	float _genTripletsMaxError = 0.0103171f;
	size_t _genTripletsNnKandidates = 14;
	size_t _genTripletsNBest = 2;
	float _smoothRadius = 0.818581f;

	ATHierarchicalClusteringTask(const ATHierarchicalClusteringTask&);
	ATHierarchicalClusteringTask operator=(const ATHierarchicalClusteringTask&);

	std::vector<ATTrajectory> useHc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<ATHit> const &hitArray, std::vector<ATHierarchicalClusteringHc::triplet> triplets, float scale) const;

	ClassDef(ATHierarchicalClusteringTask, 1);
};

#endif
