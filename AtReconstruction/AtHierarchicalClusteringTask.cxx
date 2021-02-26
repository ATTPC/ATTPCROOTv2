#include "AtHierarchicalClusteringTask.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <iostream>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "AtHierarchicalClusteringSmoothenCloud.h"
#include "AtHierarchicalClusteringHc.h"
#include "AtFindVertex.h"

// ROOT classes
#include "TClonesArray.h"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"
#include "FairLogger.h"

static void HitArrayToPointCloud(std::vector<AtHit> const &hitArray, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
   for (auto const &point : hitArray) {
      TVector3 const pointPos = point.GetPosition();

      pcl::PointXYZI pclPoint;
      pclPoint.x = pointPos.X();
      pclPoint.y = pointPos.Y();
      pclPoint.z = pointPos.Z();
      pclPoint.intensity = static_cast<float>(point.GetCharge());

      cloud->push_back(pclPoint);
   }
}

/*static void ColorByTrajectories(std::vector<AtTrajectory> const &trajectories, std::vector<AtHit> const &noMatch,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    size_t trajectoryIndex = 0;
    for (AtTrajectory const &trajectory : trajectories)
    {
        float const r = static_cast<float>((trajectoryIndex * 23) % 19) / 18.0f;
        float const g = static_cast<float>((trajectoryIndex * 23) % 7) / 6.0f;
        float const b = static_cast<float>((trajectoryIndex * 23) % 3) / 2.0f;

        AtCubicSplineFit const &cubicSplineFit = trajectory.GetCubicSplineFit();

        float const startPosition = cubicSplineFit.GetStartPosition() - 100.0f;
        float const endPosition = cubicSplineFit.GetEndPosition() + 100.0f;
        float const stepSize = (endPosition - startPosition) / 100.0f;

        for (float i = startPosition; i <= endPosition; i += stepSize)
        {
            Eigen::Vector3f const position = cubicSplineFit.CalculatePoint(i);
            pcl::PointXYZRGB point;

            point.x = position(0);
            point.y = position(1);
            point.z = position(2);
            point.r = static_cast<uint8_t>((r + 1.0f) / 2.0f * 255.0f);
            point.g = static_cast<uint8_t>((g + 1.0f) / 2.0f * 255.0f);
            point.b = static_cast<uint8_t>((b + 1.0f) / 2.0f * 255.0f);

            cloud_rgb->push_back(point);
        }

        for (AtHit const &hit : trajectory.GetHits())
        {
            TVector3 const position = hit.GetPosition();
            pcl::PointXYZRGB point;

            point.x = position.X();
            point.y = position.Y();
            point.z = position.Z();
            point.r = static_cast<uint8_t>(r * 255.0f);
            point.g = static_cast<uint8_t>(g * 255.0f);
            point.b = static_cast<uint8_t>(b * 255.0f);

            cloud_rgb->push_back(point);
        }

        ++trajectoryIndex;
    }

    // add no-matches
    for (AtHit const &hit : noMatch)
    {
        TVector3 const position = hit.GetPosition();
        pcl::PointXYZRGB point;

        point.x = position.X();
        point.y = position.Y();
        point.z = position.Z();
        point.r = 255;
        point.g = 0;
        point.b = 0;

        cloud_rgb->push_back(point);
    }
}*/

std::vector<AtTrajectory> AtHierarchicalClusteringTask::useHc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                                              std::vector<AtHit> const &hitArray,
                                                              std::vector<AtHierarchicalClusteringHc::triplet> triplets,
                                                              float scale, std::vector<AtHit> *noMatch) const
{
   AtHierarchicalClusteringHc::cluster_history result = AtHierarchicalClusteringHc::CalculateHc(
      cloud, triplets, AtHierarchicalClusteringHc::singleLinkClusterMetric,
      [&](AtHierarchicalClusteringHc::triplet const &lhs, AtHierarchicalClusteringHc::triplet const &rhs,
          pcl::PointCloud<pcl::PointXYZI>::ConstPtr) {
         float const perpendicularDistanceA =
            (rhs.center - (lhs.center + lhs.direction.dot(rhs.center - lhs.center) * lhs.direction)).squaredNorm();
         float const perpendicularDistanceB =
            (lhs.center - (rhs.center + rhs.direction.dot(lhs.center - rhs.center) * rhs.direction)).squaredNorm();

         float const angle = std::abs(std::tan(2.0f * std::acos(lhs.direction.dot(rhs.direction))));

         // squared distances!
         return std::sqrt(std::max(perpendicularDistanceA, perpendicularDistanceB)) + scale * angle;
      });

   AtHierarchicalClusteringHc::cluster_group const &clusterGroup =
      AtHierarchicalClusteringHc::GetBestClusterGroup(result, this->_bestClusterDistanceDelta);
   AtHierarchicalClusteringHc::cluster_group const &cleanedUpClusterGroup =
      AtHierarchicalClusteringHc::CleanupClusterGroup(clusterGroup, this->_cleanupMinTriplets);

   return AtHierarchicalClusteringHc::ToTrajectories(cloud, hitArray, triplets, cleanedUpClusterGroup,
                                                     this->_splineTangentScale, this->_splineMinControlPointDistance,
                                                     this->_splineJump, noMatch);
}

AtHierarchicalClusteringTask::AtHierarchicalClusteringTask() : FairTask("AtHierarchicalClusteringTask")
{
   fLogger = FairLogger::GetLogger();
   fLogger->Debug(MESSAGE_ORIGIN, "Defaul Constructor of AtHierarchicalClusteringTask");
   fPar = NULL;
   kIsPersistence = kFALSE;
}

AtHierarchicalClusteringTask::~AtHierarchicalClusteringTask()
{
   fLogger->Debug(MESSAGE_ORIGIN, "Destructor of AtHierarchicalClusteringTask");
}

void AtHierarchicalClusteringTask::SetPersistence(Bool_t value)
{
   kIsPersistence = value;
}

void AtHierarchicalClusteringTask::SetParContainers()
{
   fLogger->Debug(MESSAGE_ORIGIN, "SetParContainers of AtHierarchicalClusteringTask");

   FairRun *run = FairRun::Instance();
   if (!run)
      fLogger->Fatal(MESSAGE_ORIGIN, "No analysis run!");

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      fLogger->Fatal(MESSAGE_ORIGIN, "No runtime database!");

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      fLogger->Fatal(MESSAGE_ORIGIN, "AtDigiPar not found!!");
}

InitStatus AtHierarchicalClusteringTask::Init()
{
   fLogger->Debug(MESSAGE_ORIGIN, "Initilization of AtHierarchicalClusteringTask");

   fHierarchicalClusteringArray = new TClonesArray("AtHierarchicalClusteringHc");

   // Get a handle from the IO manager
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
      return kERROR;
   }

   fEventHArray = (TClonesArray *)ioMan->GetObject("AtEventH");
   if (fEventHArray == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find AtEvent array!");
      return kERROR;
   }

   ioMan->Register("AtHierarchicalClusteringHc", "AtTPC", fHierarchicalClusteringArray, kIsPersistence);

   return kSUCCESS;
}

InitStatus AtHierarchicalClusteringTask::ReInit()
{
   fLogger->Debug(MESSAGE_ORIGIN, "Initilization of AtHierarchicalClusteringTask");

   return kSUCCESS;
}

void AtHierarchicalClusteringTask::Exec(Option_t *option)
{
   fLogger->Debug(MESSAGE_ORIGIN, "Exec of AtHierarchicalClusteringTask");

   fHierarchicalClusteringArray->Delete();

   if (fEventHArray->GetEntriesFast() == 0)
      return;

   std::vector<AtHit> hitArray;
   AtEvent &event = *((AtEvent *)fEventHArray->At(0));
   hitArray = *event.GetHitArray();

   std::cout << "  -I- AtHierarchicalClusteringTask -  Event Number :  " << event.GetEventID() << "\n";

   try {
      // a place for no-matches
      // (points that don't belong to any trajectory)
      std::vector<AtHit> noMatch;

      std::vector<AtTrajectory> trajectories = AnalyzePointArray(hitArray, &noMatch);

   } catch (std::runtime_error e) {
      std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
   }

   // fEvent  = (AtEvent *) fEventHArray -> At(0);
}

void AtHierarchicalClusteringTask::Finish()
{
   fLogger->Debug(MESSAGE_ORIGIN, "Finish of AtHierarchicalClusteringTask");
}

std::vector<AtTrajectory>
AtHierarchicalClusteringTask::AnalyzePointArray(std::vector<AtHit> const &hitArray, std::vector<AtHit> *noMatch) const
{
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti(new pcl::PointCloud<pcl::PointXYZI>());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());

   HitArrayToPointCloud(hitArray, cloud_xyzti);
   copyPointCloud(*cloud_xyzti, *cloud_xyz);

   if (cloud_xyz->size() == 0)
      throw std::runtime_error("Empty cloud!");
   else {
      // smoothen cloud
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti_smooth(new pcl::PointCloud<pcl::PointXYZI>());

      // cloud_smooth = smoothenCloud(cloud_filtered, 12); // k nearest neighbour
      cloud_xyzti_smooth =
         AtHierarchicalClusteringSmoothenCloud::SmoothenCloud(cloud_xyzti, this->_smoothRadius); // radius

      // calculate cluster
      std::vector<AtHierarchicalClusteringHc::triplet> triplets = AtHierarchicalClusteringHc::GenerateTriplets(
         cloud_xyzti_smooth, this->_genTripletsNnKandidates, this->_genTripletsNBest, this->_genTripletsMaxError);
      // TODO: evaluate tradeoff
      // using the smooth cloud yields better curvature, but the curve is too short.
      // std::vector<AtTrajectory> trajectories = this->useHc(cloud_xyzti, hitArray, triplets,
      // this->_cloudScaleModifier, noMatch);
      std::vector<AtTrajectory> trajectories =
         this->useHc(cloud_xyzti_smooth, hitArray, triplets, this->_cloudScaleModifier, noMatch);

      return trajectories;
   }
}

/*void AtHierarchicalClusteringTask::Visualize(std::vector<AtTrajectory> const &trajectories, std::vector<AtHit> const
&noMatch) const
{
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = nullptr;
    this->AtHierarchicalClusteringTask::Visualize(trajectories, noMatch, viewer);
    viewer->close();
}*/

/*void AtHierarchicalClusteringTask::Visualize(std::vector<AtTrajectory> const &trajectories, std::vector<AtHit> const
&noMatch, std::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) const
{
    if (viewer == nullptr)
    {
        viewer = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PCL
Viewer")); viewer->setPosition(100, 100); viewer->setSize(800, 600); viewer->setBackgroundColor(1.0, 1.0, 1.0);
        viewer->setCameraPosition(0.0, 0.0, 5000.0, 0.0, 1.0, 0.0);
        viewer->setCameraClipDistances(std::numeric_limits<double>::min(), std::numeric_limits<double>::max());
        viewer->setCameraFieldOfView(3.14f / 4.0f); // 45 deg
    }

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->removeAllCoordinateSystems();

    viewer->addCoordinateSystem(500.0f, Eigen::Affine3f());

    // color-code
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    ColorByTrajectories(trajectories, noMatch, cloud_xyzrgb);

    // visualize
    size_t lineId = 0;
    auto addNewLine = [&](pcl::PointXYZ const &lhs, pcl::PointXYZ const &rhs, double r, double g, double b)
    {
        std::ostringstream oss;
        oss << "line" << lineId;
        viewer->addLine<pcl::PointXYZ>(lhs, rhs, oss.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, oss.str());
        ++lineId;
    };

    for (AtTrajectory const &trajectory : trajectories)
    {
        AtCubicSplineFit const &cubicSplineFit = trajectory.GetCubicSplineFit();

        pcl::PointXYZ start;
        start.x = trajectory.GetCentroidPoint()(0);
        start.y = trajectory.GetCentroidPoint()(1);
        start.z = trajectory.GetCentroidPoint()(2);

        pcl::PointXYZ endA;
        Eigen::Vector3f endEigA = trajectory.GetCentroidPoint() + (cubicSplineFit.GetStartPosition() *
trajectory.GetMainDirection()); endA.x = endEigA(0); endA.y = endEigA(1); endA.z = endEigA(2);

        addNewLine(start, endA, 1.0, 0.0, 1.0);

        pcl::PointXYZ endB;
        Eigen::Vector3f endEigB = trajectory.GetCentroidPoint() + (cubicSplineFit.GetEndPosition() *
trajectory.GetMainDirection()); endB.x = endEigB(0); endB.y = endEigB(1); endB.z = endEigB(2);

        addNewLine(start, endB, 1.0, 0.0, 0.0);



        // test angle
        //AtCubicSplineFit const &cubicSplineFit = trajectory.GetCubicSplineFit();

        //float const startPosition = cubicSplineFit.GetStartPosition();
        //float const endPosition = cubicSplineFit.GetEndPosition();
        //float const stepSize = (endPosition - startPosition) / 250.0f;
        //float const delta = stepSize;

        //for (float position = startPosition; position <= endPosition; position += stepSize)
        //{
        //    pcl::PointXYZ pointPcl;
        //    Eigen::Vector3f const point = cubicSplineFit.CalculatePoint(position);
        //    pointPcl.x = point(0);
        //    pointPcl.y = point(1);
        //    pointPcl.z = point(2);


        //    Eigen::Vector3f const pointLeft = cubicSplineFit.CalculatePoint(position - delta);
        //    Eigen::Vector3f const pointRight = cubicSplineFit.CalculatePoint(position + delta);

        //    Eigen::Vector3f direction = (pointLeft + ((pointRight - pointLeft) / 2.0f)) - point;
        //    direction /= direction.norm();


        //    pcl::PointXYZ targetPointPcl;
        //    Eigen::Vector3f const targetPoint = point + 20.0f * direction;
        //    targetPointPcl.x = targetPoint(0);
        //    targetPointPcl.y = targetPoint(1);
        //    targetPointPcl.z = targetPoint(2);


        //    addNewLine(pointPcl, targetPointPcl, 0.5, 0.5, 0.5);
        //}

    }

    viewer->addPointCloud(cloud_xyzrgb, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");

    viewer->spin();
}*/

void AtHierarchicalClusteringTask::SetBestClusterDistanceDelta(float value)
{
   this->_bestClusterDistanceDelta = value;
}
float AtHierarchicalClusteringTask::GetBestClusterDistanceDelta() const
{
   return this->_bestClusterDistanceDelta;
}

void AtHierarchicalClusteringTask::SetCleanupMinTriplets(size_t value)
{
   this->_cleanupMinTriplets = value;
}
size_t AtHierarchicalClusteringTask::GetCleanupMinTriplets() const
{
   return this->_cleanupMinTriplets;
}

void AtHierarchicalClusteringTask::SetCloudScaleModifier(float value)
{
   this->_cloudScaleModifier = value;
}
float AtHierarchicalClusteringTask::GetCloudScaleModifier() const
{
   return this->_cloudScaleModifier;
}

void AtHierarchicalClusteringTask::SetGenTripletsMaxError(float value)
{
   this->_genTripletsMaxError = value;
}
float AtHierarchicalClusteringTask::GetGenTripletsMaxError() const
{
   return this->_genTripletsMaxError;
}

void AtHierarchicalClusteringTask::SetGenTripletsNnKandidates(size_t value)
{
   this->_genTripletsNnKandidates = value;
}
size_t AtHierarchicalClusteringTask::GetGenTripletsNnKandidates() const
{
   return this->_genTripletsNnKandidates;
}

void AtHierarchicalClusteringTask::SetGenTripletsNBest(size_t value)
{
   this->_genTripletsNBest = value;
}
size_t AtHierarchicalClusteringTask::GetGenTripletsNBest() const
{
   return this->_genTripletsNBest;
}

void AtHierarchicalClusteringTask::SetSmoothRadius(float value)
{
   this->_smoothRadius = value;
}
float AtHierarchicalClusteringTask::GetSmoothRadius() const
{
   return this->_smoothRadius;
}

void AtHierarchicalClusteringTask::SetSplineTangentScale(float value)
{
   this->_splineTangentScale = value;
}
float AtHierarchicalClusteringTask::GetSplineTangentScale() const
{
   return this->_splineTangentScale;
}

void AtHierarchicalClusteringTask::SetSplineMinControlPointDistance(float value)
{
   this->_splineMinControlPointDistance = value;
}
float AtHierarchicalClusteringTask::GetSplineMinControlPointDistance() const
{
   return this->_splineMinControlPointDistance;
}

void AtHierarchicalClusteringTask::SetSplineJump(size_t value)
{
   this->_splineJump = value;
}
size_t AtHierarchicalClusteringTask::GetSplineJump() const
{
   return this->_splineJump;
}

ClassImp(AtHierarchicalClusteringTask)
