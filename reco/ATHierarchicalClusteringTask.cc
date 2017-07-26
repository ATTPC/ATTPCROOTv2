#define F17_VISUALIZE

#include "ATHierarchicalClusteringTask.hh"

#include <chrono>
#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#ifdef F17_VISUALIZE
#include <pcl/visualization/cloud_viewer.h>
#endif

#include "ATHierarchicalClusteringSmoothenCloud.hh"

#include "AtTpcPoint.h"
#include "FairLogger.h"

#include <iostream>

static void HitArrayToPointCloud(std::vector<ATHit> const &hitArray, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    for (auto const &point : hitArray) {
        TVector3 const pointPos = point.GetPosition();

        pcl::PointXYZI pclPoint;
        pclPoint.x = pointPos.X();
        pclPoint.y = pointPos.Y();
        pclPoint.z = pointPos.Z();
        pclPoint.intensity = (float)point.GetCharge();

        cloud->push_back(pclPoint);
    }
}

static float CalculateCloudScale(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    float totalDistance = 0.0f;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> nnIndices;
    nnIndices.reserve(2);
    std::vector<float> nnbSquaredDistances;
    nnbSquaredDistances.reserve(2);

    for (size_t pointIndex = 0; pointIndex < cloud->size(); ++pointIndex)
    {
        int const nnFound = kdtree.nearestKSearch(*cloud, (int)pointIndex, 2, nnIndices, nnbSquaredDistances);

        if (nnFound == 2)
        {
            pcl::PointXYZ const &pointA = (*cloud)[pointIndex];
            pcl::PointXYZ const &pointB = (*cloud)[nnIndices[1]];

            float const distance = std::sqrt(
                (pointA.x - pointB.x) * (pointA.x - pointB.x) +
                (pointA.y - pointB.y) * (pointA.y - pointB.y) +
                (pointA.z - pointB.z) * (pointA.z - pointB.z)
            );

            totalDistance += distance;
        }
    }

    return totalDistance / (float)cloud->size();
}

#ifdef F17_VISUALIZE
static void ColorByTrajectories(std::vector<ATTrajectory> const &trajectories, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    size_t trajectoryIndex = 0;
    for (ATTrajectory const &trajectory : trajectories)
    {
        float const r = (float)((trajectoryIndex * 23) % 19) / 18.0f;
        float const g = (float)((trajectoryIndex * 23) % 7) / 6.0f;
        float const b = (float)((trajectoryIndex * 23) % 3) / 2.0f;

        for (ATHit const &hit : trajectory.GetHits())
        {
            TVector3 position = hit.GetPosition();
            pcl::PointXYZRGB point;

            point.x = position.X();
            point.y = position.Y();
            point.z = position.Z();
            point.r = (uint8_t)(r * 255);
            point.g = (uint8_t)(g * 255);
            point.b = (uint8_t)(b * 255);

            cloud_rgb->push_back(point);
        }

        ++trajectoryIndex;
    }
}
#endif

std::vector<ATTrajectory> ATHierarchicalClusteringTask::useHc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<ATHit> const &hitArray, std::vector<ATHierarchicalClusteringHc::triplet> triplets, float scale) const
{
    ATHierarchicalClusteringHc::cluster_history result = ATHierarchicalClusteringHc::CalculateHc(cloud, triplets, ATHierarchicalClusteringHc::singleLinkClusterMetric, [&] (ATHierarchicalClusteringHc::triplet const &lhs, ATHierarchicalClusteringHc::triplet const &rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr)
    {       
        float const perpendicularDistanceA = (rhs.center - (lhs.center + lhs.direction.dot(rhs.center - lhs.center) * lhs.direction)).squaredNorm();
        float const perpendicularDistanceB = (lhs.center - (rhs.center + rhs.direction.dot(lhs.center - rhs.center) * rhs.direction)).squaredNorm();

        float const angle = std::abs(std::tan(2.0f * std::acos(lhs.direction.dot(rhs.direction))));

        // squared distances!
        return std::sqrt(std::max(perpendicularDistanceA, perpendicularDistanceB)) + scale * angle;
    });

    ATHierarchicalClusteringHc::cluster_group const &clusterGroup = ATHierarchicalClusteringHc::GetBestClusterGroup(result, this->_bestClusterDistanceDelta);
    ATHierarchicalClusteringHc::cluster_group const &cleanedUpClusterGroup = ATHierarchicalClusteringHc::CleanupClusterGroup(clusterGroup, this->_cleanupMinTriplets);

    return ToTrajectories(cloud, hitArray, triplets, cleanedUpClusterGroup);
}



ATHierarchicalClusteringTask::ATHierarchicalClusteringTask()
    : FairTask("ATHierarchicalClusteringTask")
{
    fLogger->Debug(MESSAGE_ORIGIN, "Defaul Constructor of ATHierarchicalClusteringTask");
}

ATHierarchicalClusteringTask::~ATHierarchicalClusteringTask()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Destructor of ATHierarchicalClusteringTask");
}

void ATHierarchicalClusteringTask::SetParContainers()
{
    fLogger->Debug(MESSAGE_ORIGIN, "SetParContainers of ATHierarchicalClusteringTask");

    // Load all necessary parameter containers from the runtime data base
    /*
    FairRunAna* ana = FairRunAna::Instance();
    FairRuntimeDb* rtdb=ana->GetRuntimeDb();
    <ATHierarchicalClusteringTaskDataMember> = (<ClassPointer>*)
        (rtdb->getContainer("<ContainerName>"));
    */
}

InitStatus ATHierarchicalClusteringTask::Init()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Initilization of ATHierarchicalClusteringTask");

    // Get a handle from the IO manager
    FairRootManager* ioman = FairRootManager::Instance();

    // Get a pointer to the previous already existing data level
    /*
        <InputDataLevel> = (TClonesArray*) ioman->GetObject("InputDataLevelName");
        if ( ! <InputLevel> ) {
        fLogger->Error(MESSAGE_ORIGIN,"No InputDataLevelName array!\n ATHierarchicalClusteringTask will be inactive");
        return kERROR;
        }
    */

    // Create the TClonesArray for the output data and register
    // it in the IO manager
    /*
        <OutputDataLevel> = new TClonesArray("OutputDataLevelName", 100);
        ioman->Register("OutputDataLevelName","OutputDataLevelName",<OutputDataLevel>,kTRUE);
    */

    // Do whatever else is needed at the initilization stage
    // Create histograms to be filled
    // initialize variables

    return kSUCCESS;
}

InitStatus ATHierarchicalClusteringTask::ReInit()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Initilization of ATHierarchicalClusteringTask");

    return kSUCCESS;
}

void ATHierarchicalClusteringTask::Exec(Option_t* option)
{
    fLogger->Debug(MESSAGE_ORIGIN, "Exec of ATHierarchicalClusteringTask");
}

void ATHierarchicalClusteringTask::Finish()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Finish of ATHierarchicalClusteringTask");
}


std::vector<ATTrajectory> ATHierarchicalClusteringTask::AnalyzePointArray(std::vector<ATHit> const &hitArray) const
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());

    HitArrayToPointCloud(hitArray, cloud_xyzti);
    copyPointCloud(*cloud_xyzti, *cloud_xyz);

    if (cloud_xyz->size() == 0)
        throw std::runtime_error("Empty cloud!");
    else
    {
        // calculate cloud-scale
        float const cloudScale = CalculateCloudScale(cloud_xyz);
        // std::cout << "XX cloudScale: " << cloudScale << std::endl;

        // smoothen cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti_smooth(new pcl::PointCloud<pcl::PointXYZI>());

        //cloud_smooth = smoothenCloud(cloud_filtered, 12); // k nearest neighbour
        cloud_xyzti_smooth = ATHierarchicalClusteringSmoothenCloud::SmoothenCloud(cloud_xyzti, cloudScale * this->_smoothRadius); // radius

        // calculate cluster
        std::vector<ATHierarchicalClusteringHc::triplet> triplets = ATHierarchicalClusteringHc::GenerateTriplets(cloud_xyzti_smooth, this->_genTripletsNnKandidates, this->_genTripletsNBest, this->_genTripletsMaxError);
        std::vector<ATTrajectory> trajectories = this->useHc(cloud_xyzti_smooth, hitArray, triplets, cloudScale * this->_cloudScaleModifier);

#ifdef F17_VISUALIZE
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.setPosition(100, 100);
        viewer.setSize(800, 600);
        viewer.setBackgroundColor(1.0, 1.0, 1.0);
        viewer.setCameraPosition(0.0, 0.0, 2000.0, 0.0, 1.0, 0.0);

        // color-code
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
        ColorByTrajectories(trajectories, cloud_xyzrgb);

        // visualize
        size_t lineId = 0;
        for (ATTrajectory const &trajectory : trajectories)
        {
            pcl::PointXYZ startA;
            startA.x = trajectory.GetCentroidPoint()(0);
            startA.y = trajectory.GetCentroidPoint()(1);
            startA.z = trajectory.GetCentroidPoint()(2);

            pcl::PointXYZ endA;
            Eigen::Vector3f endEigA = trajectory.GetCentroidPoint() + 1000.0f * trajectory.GetMainDirection();
            endA.x = endEigA(0);
            endA.y = endEigA(1);
            endA.z = endEigA(2);

            viewer.addLine<pcl::PointXYZ>(startA, endA, "line" + lineId);
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "line" + lineId);
            ++lineId;

            pcl::PointXYZ startB;
            startB.x = trajectory.GetCentroidPoint()(0);
            startB.y = trajectory.GetCentroidPoint()(1);
            startB.z = trajectory.GetCentroidPoint()(2);

            pcl::PointXYZ endB;
            Eigen::Vector3f endEigB = trajectory.GetCentroidPoint() - 1000.0f * trajectory.GetMainDirection();
            endB.x = endEigB(0);
            endB.y = endEigB(1);
            endB.z = endEigB(2);

            viewer.addLine<pcl::PointXYZ>(startB, endB, "line" + lineId);
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line" + lineId);
            ++lineId;
        }

        viewer.addPointCloud(cloud_xyzrgb, "cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
        viewer.addPointCloud(cloud_xyz, "cloud_original");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_original");
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cloud_original");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_original");

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(16);
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
#endif

        return trajectories;
    }
}

void ATHierarchicalClusteringTask::SetBestClusterDistanceDelta(float value) { this->_bestClusterDistanceDelta = value; }
float ATHierarchicalClusteringTask::GetBestClusterDistanceDelta() const { return this->_bestClusterDistanceDelta; }

void ATHierarchicalClusteringTask::SetCleanupMinTriplets(size_t value) { this->_cleanupMinTriplets = value; }
size_t ATHierarchicalClusteringTask::GetCleanupMinTriplets() const { return this->_cleanupMinTriplets; }

void ATHierarchicalClusteringTask::SetCloudScaleModifier(float value) { this->_cloudScaleModifier = value; }
float ATHierarchicalClusteringTask::GetCloudScaleModifier() const { return this->_cloudScaleModifier; }

void ATHierarchicalClusteringTask::SetGenTripletsMaxError(float value) { this->_genTripletsMaxError = value; }
float ATHierarchicalClusteringTask::GetGenTripletsMaxError() const { return this->_genTripletsMaxError; }

void ATHierarchicalClusteringTask::SetGenTripletsNnKandidates(size_t value) { this->_genTripletsNnKandidates = value; }
size_t ATHierarchicalClusteringTask::GetGenTripletsNnKandidates() const { return this->_genTripletsNnKandidates; }

void ATHierarchicalClusteringTask::SetGenTripletsNBest(size_t value) { this->_genTripletsNBest = value; }
size_t ATHierarchicalClusteringTask::GetGenTripletsNBest() const { return this->_genTripletsNBest; }

void ATHierarchicalClusteringTask::SetSmoothRadius(float value) { this->_smoothRadius = value; }
float ATHierarchicalClusteringTask::GetSmoothRadius() const { return this->_smoothRadius; }

ClassImp(ATHierarchicalClusteringTask)
