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

static void hitArrayToPointCloud(std::vector<ATHit> const &hitArray, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
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

static float calculateCloudScale(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
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
static void colorByCluster(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, ATHierarchicalClusteringCluster const &cluster, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    cloud_rgb->clear();
    copyPointCloud(*cloud, *cloud_rgb);

    // default color: red
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        auto &point = (*cloud_rgb)[i];

        point.r = 255;
        point.g = 0;
        point.b = 0;
    }

    size_t clusterIndex = 0;
    for (std::vector<size_t> const &pointIndices : cluster.getClusters())
    {
        double const r = (double)((clusterIndex * 23) % 19) / 18.0;
        double const g = (double)((clusterIndex * 23) % 7) / 6.0;
        double const b = (double)((clusterIndex * 23) % 3) / 2.0;

        for (size_t index : pointIndices)
        {
            auto &point = (*cloud_rgb)[index];

            point.r = (uint8_t)(r * 255);
            point.g = (uint8_t)(g * 255);
            point.b = (uint8_t)(b * 255);
        }

        ++clusterIndex;
    }
}
#endif

ATHierarchicalClusteringCluster ATHierarchicalClusteringTask::useHc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<ATHierarchicalClusteringHc::triplet> triplets, float scale) const
{
    ATHierarchicalClusteringHc::cluster_history result = ATHierarchicalClusteringHc::calculateHc(cloud, triplets, ATHierarchicalClusteringHc::singleLinkClusterMetric, [&] (ATHierarchicalClusteringHc::triplet const &lhs, ATHierarchicalClusteringHc::triplet const &rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr)
    {       
        float const perpendicularDistanceA = (rhs.center - (lhs.center + lhs.direction.dot(rhs.center - lhs.center) * lhs.direction)).squaredNorm();
        float const perpendicularDistanceB = (lhs.center - (rhs.center + rhs.direction.dot(lhs.center - rhs.center) * rhs.direction)).squaredNorm();

        float const angle = std::abs(std::tan(2.0f * std::acos(lhs.direction.dot(rhs.direction))));

        // squared distances!
        return std::sqrt(std::max(perpendicularDistanceA, perpendicularDistanceB)) + scale * angle;
    });

    ATHierarchicalClusteringHc::cluster_group const &clusterGroup = ATHierarchicalClusteringHc::getBestClusterGroup(result, this->_bestClusterDistanceDelta);
    ATHierarchicalClusteringHc::cluster_group const &cleanedUpClusterGroup = ATHierarchicalClusteringHc::cleanupClusterGroup(clusterGroup, this->_cleanupMinTriplets);

    return ATHierarchicalClusteringHc::toCluster(triplets, cleanedUpClusterGroup, cloud->size());
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


ATHierarchicalClusteringCluster ATHierarchicalClusteringTask::AnalyzePointArray(std::vector<ATHit> const &hitArray) const
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());

    hitArrayToPointCloud(hitArray, cloud_xyzti);
    copyPointCloud(*cloud_xyzti, *cloud_xyz);

    if (cloud_xyz->size() == 0)
        throw std::runtime_error("Empty cloud!");
    else
    {
        // calculate cloud-scale
        float const cloudScale = calculateCloudScale(cloud_xyz);
        // std::cout << "XX cloudScale: " << cloudScale << std::endl;

        // smoothen cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti_smooth(new pcl::PointCloud<pcl::PointXYZI>());

        //cloud_smooth = smoothenCloud(cloud_filtered, 12); // k nearest neighbour
        cloud_xyzti_smooth = ATHierarchicalClusteringSmoothenCloud::smoothenCloud(cloud_xyzti, cloudScale * this->_smoothRadius); // radius

        // calculate cluster
        std::vector<ATHierarchicalClusteringHc::triplet> triplets = ATHierarchicalClusteringHc::generateTriplets(cloud_xyzti_smooth, this->_genTripletsNnKandidates, this->_genTripletsNBest, this->_genTripletsMaxError);
        ATHierarchicalClusteringCluster cluster = this->useHc(cloud_xyzti_smooth, triplets, cloudScale * this->_cloudScaleModifier);

#ifdef F17_VISUALIZE
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.setPosition(100, 100);
        viewer.setSize(800, 600);
        viewer.setBackgroundColor(1.0, 1.0, 1.0);
        viewer.setCameraPosition(0.0, 0.0, 2000.0, 0.0, 1.0, 0.0);

        // color-code
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
        colorByCluster(cloud_xyz, cluster, cloud_xyzrgb);

        // visualize
        viewer.addPointCloud(cloud_xyzrgb, "cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(16);
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
#endif

        return cluster;
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
