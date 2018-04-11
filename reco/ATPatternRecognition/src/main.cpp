#pragma warning(push, 0)
//#define TIME  // Uncomment this for time mesurement (output to stderr)
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifdef TIME
#include <time.h>
#endif
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#pragma warning(pop)

#include "hc.h"
#include "msd.h"
#include "mst.h"
#include "smoothenCloud.h"

// usage message
const char *usage =
    "Usage:\n"
    "\tcloud_viewer [options] <infile>\n"
    "Options (defaults in brackets):\n"
    "\t-gnuplot       print result as a gnuplot command\n"
    "\t-r <radius>    radius for point smoothing [7.0]\n"
    "\t               (when negative, it is set to dNN)\n"
    "\t-k <n>         number of nearest neighbours used for creating triplets\n"
    "\t               [19]\n"
    "\t-n <n>         number of the best triplets to use [3]\n"
    "\t-a <alpha>     maximum angle between the lines in a triplet [0.015]\n"
    "\t-s <scale>     scalingfactor for clustering [1.0]\n"
    "\t-t <dist>      best cluster distance [5.0]\n"
    "\t-m <n>         minimum number of triplets for a cluster [5]\n"
    "\t-o <outfile>   write results to <outfile>\n"
    "\t-x11           show interactive window with clustering\n"
    "\t-v <n>         be verbose and save debug trace to file. n is the\n"
    "\t               verbose depth\n"
    "Version:\n"
    "\t1.0 from 2018-03-27";
/*
    "\t-dx <dx>       step width in x'y'-plane [0]\n"
    "\t               when <dx>=0, it is set to 1/64 of total width\n"
    "\t-nlines <nl>   maximum number of lines returned [0]\n"
    "\t               when <nl>=0, all lines are returned\n"
    "\t-minvotes <nv> only lines with at least <nv> points are returned [0]\n"
    "\t-raw           print plot data in easily machine-parsable format\n"
    "\t-vv            be even more verbose\n";
*/

struct hc_params {
  float cloudScaleModifier;
  size_t genTripletsNnKandidates;
  size_t genTripletsNBest;
  size_t cleanupMinTriplets;
  float smoothRadius;
  float genTripletsMaxError;
  float bestClusterDistanceDelta;
  float _padding;
};

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_t;

struct Point {
  float x;
  float y;
  float z;
  std::vector<int> clIds;

  Point(pcl::PointXYZ point) {
    x = point.x;
    y = point.y;
    z = point.z;
  }

  bool operator==(const Point &p) const {
    return (x == p.x && y == p.y && z == p.z);
  }
};

void visualizeTriplets(pcl::visualization::PCLVisualizer &viewer,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       std::vector<hc::triplet> const &triplets) {
  size_t lineId = 0;

  for (size_t tripletIndex = 0; tripletIndex < triplets.size();
       ++tripletIndex) {
    hc::triplet const &tripletEl = triplets[tripletIndex];
    double const r = (double)((tripletIndex * 23) % 19) / 18.0;
    double const g = (double)((tripletIndex * 23) % 7) / 6.0;
    double const b = (double)((tripletIndex * 23) % 3) / 2.0;

    {
      pcl::PointXYZI const &pointA = (*cloud)[tripletEl.pointIndexA];
      pcl::PointXYZI const &pointB = (*cloud)[tripletEl.pointIndexB];

      std::ostringstream oss;
      oss << "line" << lineId;
      viewer.addLine<pcl::PointXYZI>(pointA, pointB, r, g, b, oss.str());
      ++lineId;
    }

    {
      pcl::PointXYZI const &pointA = (*cloud)[tripletEl.pointIndexB];
      pcl::PointXYZI const &pointB = (*cloud)[tripletEl.pointIndexC];

      std::ostringstream oss;
      oss << "line" << lineId;
      viewer.addLine<pcl::PointXYZI>(pointA, pointB, r, g, b, oss.str());
      ++lineId;
    }

    ++tripletIndex;
  }
}

void visualizeClusterAsMst(pcl::visualization::PCLVisualizer &viewer,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                           Cluster const &cluster) {
  size_t lineId = 0;
  std::vector<pcl::PointIndicesPtr> clusters = cluster.getClusters();

  for (size_t clusterIndex = 0; clusterIndex < clusters.size();
       ++clusterIndex) {
    pcl::PointIndicesPtr const &pointIndices = clusters[clusterIndex];
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(pointIndices);
    extract.setNegative(false);
    extract.filter(*clusterCloud);

    std::vector<mst::state> mstResult =
        mst::calculateMinimumSpanningTree(clusterCloud);

    if (mstResult.size() < 1) continue;

    double const r = (double)((clusterIndex * 23) % 19) / 18.0;
    double const g = (double)((clusterIndex * 23) % 7) / 6.0;
    double const b = (double)((clusterIndex * 23) % 3) / 2.0;

    mst::state const &lastMstResult = mstResult[mstResult.size() - 1];
    std::vector<mst::edge> const &edges = lastMstResult.edges;
    for (size_t i = 0; i < edges.size(); ++i) {
      mst::edge const &edge = edges[i];
      pcl::PointXYZI const &pointA = (*clusterCloud)[edge.voxelIndexA];
      pcl::PointXYZI const &pointB = (*clusterCloud)[edge.voxelIndexB];

      std::ostringstream oss;
      oss << "line" << lineId;
      viewer.addLine<pcl::PointXYZI>(pointA, pointB, r, g, b, oss.str());
      ++lineId;
    }
  }
}

/*
Cluster useHc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
              std::vector<hc::triplet> triplets, float scale,
              float bestClusterDistanceDelta, size_t cleanupMinTriplets,
              int opt_verbose = 0) {
  hc::ScaleTripletMetric scaleTripletMetric(scale);
  hc::cluster_history result =
      hc::calculateHc(cloud, triplets, scaleTripletMetric,
                      hc::singleLinkClusterMetric, opt_verbose);
  hc::cluster_group const &clusterGroup =
      hc::getBestClusterGroup(result, bestClusterDistanceDelta);
  hc::cluster_group const &cleanedUpClusterGroup =
      hc::cleanupClusterGroup(clusterGroup, cleanupMinTriplets);

  return hc::toCluster(triplets, cleanedUpClusterGroup, cloud->size());
}
*/

/*
@brief cluster pointcloud.

This function clusters the pointcloud and returns the clusters.

@param  cloud the point cloud
@param  triplets the triplets
@param  scale the scale for the triplet metric
@param  cdist the max distance for merging clusters
@param  cleanup_min_triplets the minimum count of triplets for a cluster
@param  opt_verbose verbosity
@return the clusters
*/
Cluster use_hc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
               std::vector<hc::triplet> triplets, float scale, float cdist,
               size_t cleanup_min_triplets, int opt_verbose = 0) {
  hc::ScaleTripletMetric scale_triplet_metric(scale);
  hc::cluster_group result =
      hc::compute_hc(cloud, triplets, scale_triplet_metric, cdist, opt_verbose);
  hc::cluster_group const &cleaned_up_cluster_group =
      hc::cleanupClusterGroup(result, cleanup_min_triplets);

  return hc::toCluster(triplets, cleaned_up_cluster_group, cloud->size());
}

void colorByIndex(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb) {
  cloud_rgb->clear();
  pcl::copyPointCloud(*cloud, *cloud_rgb);

  size_t index = 0;
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud_rgb->begin();
       it != cloud_rgb->end(); ++it) {
    it->r = (uint8_t)(256 * (index % 8) / 8);
    it->g = it->r / 2;
    it->b = it->r;
    ++index;
  }
}

void colorByIntensity(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
                      float minIntensity = 0.0f, float maxIntensity = 4000.0f) {
  cloud_rgb->clear();
  pcl::copyPointCloud(*cloud, *cloud_rgb);

  float size = maxIntensity - minIntensity;

  for (size_t i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZRGB &point = (*cloud_rgb)[i];

    point.r = (uint8_t)(256 * (((*cloud)[i].intensity - minIntensity) / size));
    point.g = point.r / 2;
    point.b = point.r;
  }
}

void colorByCluster(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
                    Cluster const cluster,
                    std::vector<rgb_t> const cluster_rgb) {
  cloud_rgb->clear();
  pcl::copyPointCloud(*cloud, *cloud_rgb);

  // default color: red
  for (size_t i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZRGB &point = (*cloud_rgb)[i];

    point.r = 255;
    point.g = 0;
    point.b = 0;
  }

  std::vector<pcl::PointIndicesPtr> clusters = cluster.getClusters();
  for (size_t i = 0; i < clusters.size(); ++i) {
    pcl::PointIndicesPtr const &pointIndices = clusters[i];
    rgb_t rgb = cluster_rgb[i];
    std::vector<int> indices = pointIndices->indices;
    for (size_t i = 0; i < indices.size(); ++i) {
      int index = indices[i];
      pcl::PointXYZRGB &point = (*cloud_rgb)[index];

      point.r = rgb.r;
      point.g = rgb.g;
      point.b = rgb.b;
    }
  }
}

std::vector<rgb_t> createClusterColour(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, Cluster const cluster) {
  std::vector<rgb_t> cluster_rgb;
  std::vector<pcl::PointIndicesPtr> clusters = cluster.getClusters();
  for (size_t clusterIndex = 0; clusterIndex < clusters.size();
       ++clusterIndex) {
    pcl::PointIndicesPtr const &pointIndices = clusters[clusterIndex];
    double const r = (double)((clusterIndex * 23) % 19) / 18.0;
    double const g = (double)((clusterIndex * 23) % 7) / 6.0;
    double const b = (double)((clusterIndex * 23) % 3) / 2.0;
    rgb_t rgb;
    rgb.r = (uint8_t)(r * 255);
    rgb.g = (uint8_t)(g * 255);
    rgb.b = (uint8_t)(b * 255);

    cluster_rgb.push_back(rgb);
  }
  return cluster_rgb;
}

/*
@brief prints gnuplot output.

This function prints a cloud with clustering as gnuplot script to stdout.

@param  cloud   the point cloud
@param  cluster the clusters
@return vector of mean squared distances of every point in the cloud
*/
void clustersToGnuplot(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       Cluster const cluster, std::vector<rgb_t> cluster_rgb) {
  // gnuplot with RGB as Hex. The colour represents the point's cluster
  std::ostringstream oss, header;
  pcl::PointXYZ min, max;
  header << "set mouse\n";
  pcl::getMinMax3D(*cloud, min, max);
  header << "set xrange [" << min.x << ":" << max.x << "]\n";
  header << "set yrange [" << min.y << ":" << max.y << "]\n";
  header << "set zrange [" << min.z << ":" << max.z << "]\nsplot ";

  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points =
      cloud->points;
  std::vector<pcl::PointIndicesPtr> clusters = cluster.getClusters();
  for (size_t clusterIndex = 0; clusterIndex < clusters.size();
       ++clusterIndex) {
    pcl::PointIndicesPtr const &pointIndices = clusters[clusterIndex];
    // get color colour
    rgb_t rgb = cluster_rgb[clusterIndex];
    unsigned long rgb_hex = (rgb.r << 16) | (rgb.g << 8) | rgb.b;

    header << " '-' with points lc '#" << std::hex << rgb_hex
           << "' title 'track " << clusterIndex << "',";
    for (size_t i = 0; i < pointIndices->indices.size(); ++i) {
      int index = pointIndices->indices[i];
      pcl::PointXYZ point = cloud->points[index];

      // remove clustered points from point-vector
      for (std::vector<pcl::PointXYZ,
                       Eigen::aligned_allocator<pcl::PointXYZ> >::iterator it =
               points.end();
           it != points.begin(); --it) {
        if (it->x == point.x && it->y == point.y && it->z == point.z) {
          points.erase(it);
          break;
        }
      }
      oss << point.x << " " << point.y << " " << point.z << std::endl;
    }
    oss << "e" << std::endl;
  }

  // plot all unclustered points red
  header << "'-' with points lc 'red' title 'noise'\n";
  for (std::vector<pcl::PointXYZ,
                   Eigen::aligned_allocator<pcl::PointXYZ> >::iterator it =
           points.begin();
       it != points.end(); ++it) {
    oss << it->x << " " << it->y << " " << it->z << std::endl;
  }
  oss << "e\n";

  oss << "pause mouse keypress\n";
  std::cout << header.str() << oss.str();
}

/*
@brief prints gnuplot output.

This function prints a coloured cloud as gnuplot script to stdout.

@param  cloud   the point cloud
@return vector of mean squared distances of every point in the cloud
*/
void clustersToGnuplot(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  // Method for gnuplot 4.6 with RGB vale from RGB Pointcloud
  std::ostringstream oss;
  oss << "set mouse\nrgb(r,g,b) = 65536 * int(r) + 256 * int(g) + int(b)\n";
  oss << "splot '-' using 1:2:3:(rgb($4,$5,$6)) with points lc rgb variable\n";
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin();
       it != cloud->end(); ++it) {
    oss << it->x << " " << it->y << " " << it->z << " ";
    oss << (int)it->r << " " << (int)it->g << " " << (int)it->b << "\n";
  }
  oss << "e\n";
  oss << "pause mouse keypress\n";
  std::cout << oss.str();
}

/*
@brief saves the smoothen cloud as gnuplot-file

@param  cloud   the point cloud
@param  fname   filename
@return vector of mean squared distances of every point in the cloud
*/
void debugGnuplot(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_smooth,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                  const char *fname = "debug_smoothed.gnuplot") {
  // saves cloud in gnuplot file
  std::ofstream of(fname);
  pcl::PointXYZI min, max;
  if (!of.is_open()) {
    std::cerr << "Could Not save under '" << fname << std::endl;
    ;
    exit(1);
  }
  pcl::getMinMax3D(*cloud, min, max);
  of << "set mouse\n";
  of << "set xrange [" << min.x << ":" << max.x << "]\n";
  of << "set yrange [" << min.y << ":" << max.y << "]\n";
  of << "set zrange [" << min.z << ":" << max.z << "]\n";

  of << "splot '-' with points lc 'black' title 'original', '-' with points lc "
        "'red' title 'smoothed'\n";
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZI point = cloud->points[i];
    of << point.x << " " << point.y << " " << point.z << std::endl;
  }
  of << "e\n";

  for (size_t i = 0; i < cloud_smooth->points.size(); ++i) {
    pcl::PointXYZI point = cloud_smooth->points[i];
    of << point.x << " " << point.y << " " << point.z << std::endl;
  }
  of << "e\npause mouse keypress\n";

  of.close();
}

/*
@brief Saves point cloud with clusters as csv

This function saves a point cloud with clusterinformation in the following
format:
  x,y,z,cluster

@param  cloud   the point cloud
@param  cluster the clusters
@param  fname   filename
@return vector of mean squared distances of every point in the cloud
*/
void clustersToCSV(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   Cluster const cluster) {
  std::vector<Point> points;

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZ point = cloud->points[i];
    points.push_back(Point(point));
  }

  std::vector<pcl::PointIndicesPtr> clusters = cluster.getClusters();
  for (size_t clusterIndex = 0; clusterIndex < clusters.size();
       ++clusterIndex) {
    pcl::PointIndicesPtr const &pointIndices = clusters[clusterIndex];
    for (size_t i = 0; i < pointIndices->indices.size(); ++i) {
      int index = pointIndices->indices[i];
      points[index].clIds.push_back(clusterIndex);
    }
  }
  
  std::cout << "# Comment: a trackID of -1 represents noise\n# x, y, z, trackID\n";

  for (std::vector<Point>::iterator it = points.begin(); it != points.end();
       ++it) {
    std::cout << it->x << "," << it->y << "," << it->z << ",";
    if (it->clIds.empty()) {
      // Noise
      std::cout << "-1\n";
    } else {
      for (size_t i = 0; i < it->clIds.size(); ++i) {
        if (i > 0) {
          std::cout << ";";
        }
        std::cout << it->clIds[i];
      }
      std::cout << std::endl;
    }
  }
}

/*
@brief saves a point cloud as csv

@param  cloud   the point cloud
@param  fname   filename
@return vector of mean squared distances of every point in the cloud
*/
void cloudToCSV(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                const char *fname = "debug_smoothed.csv") {
  // converts pointcloud to csv
  std::ofstream of(fname);
  if (!of.is_open()) {
    std::cerr << "Could Not save under '" << fname << std::endl;
    exit(1);
  }
  of << "# x,y,z" << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZI point = cloud->points[i];
    of << point.x << "," << point.y << "," << point.z << std::endl;
  }
  of.close();
}

int main(int argc, char **argv) {
#ifdef TIME
  double time1 = 0.0, tstart;
#endif
  // default values for command line options
  char *infile_name = NULL, *outfile_name = NULL;
  int opt_verbose = 0;
  bool gnuplot = false, x11 = false;
  hc_params opt_params;

  hc_params bestParams;
  // ATTPC
  // Skalierungsfaktor für das clustering
  bestParams.cloudScaleModifier = 1.0;
  // Radius zum glätten der Punkte
  bestParams.smoothRadius = 7.0f;
  // Anzal nächste Nachbarn um Triplets zu erzeugen
  bestParams.genTripletsNnKandidates = 19;
  // Anzahl bester Triplets, die aus der Kandidatenmenge übernommen werden
  bestParams.genTripletsNBest = 3;
  // Vermutung: Maximaler Winkel zwichen den Geraden AB und BC im Triplet.
  bestParams.genTripletsMaxError = 0.015;
  // Schwellwert für den besten Cluster Abstand.
  // Darüber werden die Cluster nicht mehr vereinigt
  bestParams.bestClusterDistanceDelta = 5.0;  // Aymans: 2.0
  // Schwellwert zum Entfernen aller Cluster,
  // welche weniger Tripletten besitzen als angegeben.
  bestParams.cleanupMinTriplets = 5;
  opt_params = bestParams;

  // parse command line
  for (int i = 1; i < argc; i++) {
    if (0 == strcmp(argv[i], "-v")) {
      ++i;
      if (i < argc) {
        opt_verbose = atoi(argv[i]);
      } else {
        std::cerr << usage << std::endl;
        return 1;
      }
    } else if (0 == strcmp(argv[i], "-gnuplot")) {
      gnuplot = true;
    } else if (0 == strcmp(argv[i], "-s")) {
      ++i;
      if (i < argc) {
        opt_params.cloudScaleModifier = atof(argv[i]);
      } else {
        std::cerr << usage << std::endl;
        return 1;
      }
    } else if (0 == strcmp(argv[i], "-r")) {
      ++i;
      if (i < argc) {
        opt_params.smoothRadius = atof(argv[i]);
      } else {
        std::cerr << usage << std::endl;
        return 1;
      }
    } else if (0 == strcmp(argv[i], "-k")) {
      ++i;
      if (i < argc) {
        opt_params.genTripletsNnKandidates = atoi(argv[i]);
      } else {
        std::cerr << usage << std::endl;
        return 1;
      }
    } else if (0 == strcmp(argv[i], "-n")) {
      ++i;
      if (i < argc) {
        opt_params.genTripletsNBest = atoi(argv[i]);
      } else {
        std::cerr << usage << std::endl;
        return 1;
      }
    } else if (0 == strcmp(argv[i], "-a")) {
      ++i;
      if (i < argc) {
        opt_params.genTripletsMaxError = atof(argv[i]);
      } else {
        std::cerr << usage << std::endl;
        return 1;
      }
    } else if (0 == strcmp(argv[i], "-t")) {
      ++i;
      if (i < argc) {
        opt_params.bestClusterDistanceDelta = atof(argv[i]);
      } else {
        std::cerr << usage << std::endl;
        return 1;
      }
    } else if (0 == strcmp(argv[i], "-m")) {
      ++i;
      if (i < argc) {
        opt_params.cleanupMinTriplets = atoi(argv[i]);
      } else {
        std::cerr << usage << std::endl;
        return 1;
      }
    } else if (0 == strcmp(argv[i], "-o")) {
      if (i + 1 == argc) {
        std::cerr << "Not enough parameters\n" << usage << std::endl;
        return 1;
      } else if (argv[i + 1][0] == '-') {
        std::cerr << "Please enter outfile name\n" << usage << std::endl;
        return 1;
      }
      outfile_name = argv[++i];
    } else if (0 == strcmp(argv[i], "-x11")) {
      x11 = true;
    } else if (argv[i][0] == '-') {
      std::cerr << usage << std::endl;
      return 1;
    } else {
      infile_name = argv[i];
    }
  }

  // plausibilty checks
  if (!infile_name) {
    std::cerr << "Error: no infile given!\n" << usage << std::endl;
    return 1;
  }
  if (0 != access(infile_name, R_OK)) {
    std::cerr << "Error: cannot read infile '" << infile_name << "'!\n" << usage << std::endl;
    return 1;
  }

// load data
#ifdef TIME
  tstart = clock();
#endif
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::io::loadPCDFile(std::string(infile_name), *cloud_xyzti);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud_xyzti, *cloud_xyz);
  if (cloud_xyz->size() == 0) {
    std::cerr << "Error: empty cloud in file '" << infile_name << "'\n";
    return 2;
  }
#ifdef TIME
  time1 = clock() - tstart;
  std::cerr << "file loading time: " << time1 / CLOCKS_PER_SEC << "s"
            << std::endl;
#endif

  if (opt_params.smoothRadius < 0.0) {
    float fq = msd::first_quartile(cloud_xyz);
    opt_params.smoothRadius = fq;
    if (opt_verbose > 0) {
      std::cout << "Computed smoothen radius: " << fq << std::endl;
    }
  }
// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new
// pcl::PointCloud<pcl::PointXYZI>());  pcl::PointCloud<pcl::PointXYZ>::Ptr
// cloud_outliner(new pcl::PointCloud<pcl::PointXYZ>());

/*
// outliner removal
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(true);
sorfilter.setInputCloud(cloud_xyz);
sorfilter.setMeanK(12);
sorfilter.setStddevMulThresh(2.0);
sorfilter.filter(*cloud_filtered);

// get removed indices
pcl::IndicesConstPtr indices_rem = sorfilter.getRemovedIndices();
pcl::ExtractIndices<pcl::PointXYZ> extract;
extract.setInputCloud(cloud_xyz);
extract.setIndices(indices_rem);
extract.setNegative(false);
extract.filter(*cloud_outliner);
*/

// smoothen cloud
#ifdef TIME
  tstart = clock();
#endif
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti_smooth(
      new pcl::PointCloud<pcl::PointXYZI>());

  // cloud_smooth = smoothenCloud(cloud_filtered, 12); // k nearest neighbour
  cloud_xyzti_smooth =
      smoothenCloud(cloud_xyzti, opt_params.smoothRadius);  // radius

  if (opt_verbose > 0) {
    cloudToCSV(cloud_xyzti_smooth);
    debugGnuplot(cloud_xyzti_smooth, cloud_xyzti);
  }
#ifdef TIME
  time1 = clock() - tstart;
  std::cerr << "smoothen time: " << time1 / CLOCKS_PER_SEC << "s" << std::endl;
#endif

  // calculate cluster
  Cluster cluster;
  std::vector<hc::triplet> triplets;

#ifdef TIME
  tstart = clock();
#endif
  triplets = hc::generateTriplets(
      cloud_xyzti_smooth, opt_params.genTripletsNnKandidates,
      opt_params.genTripletsNBest, opt_params.genTripletsMaxError);
#ifdef TIME
  time1 = clock() - tstart;
  std::cerr << "gen triplet time: " << time1 / CLOCKS_PER_SEC << "s"
            << std::endl;
#endif

#ifdef TIME
  tstart = clock();
#endif
  /*
    cluster = useHc(cloud_xyzti_smooth, triplets, opt_params.cloudScaleModifier,
                    opt_params.bestClusterDistanceDelta,
                    opt_params.cleanupMinTriplets, opt_verbose);
  */
  cluster = use_hc(cloud_xyzti_smooth, triplets, opt_params.cloudScaleModifier,
                   opt_params.bestClusterDistanceDelta,
                   opt_params.cleanupMinTriplets, opt_verbose);
#ifdef TIME
  time1 = clock() - tstart;
  std::cerr << "clustering time: " << time1 / CLOCKS_PER_SEC << "s"
            << std::endl;
#endif

  // create colours by clusters
  std::vector<rgb_t> cluster_rgb = createClusterColour(cloud_xyz, cluster);

  if (gnuplot) {
    clustersToGnuplot(cloud_xyz, cluster, cluster_rgb);
  } else if (outfile_name) {
    // redirect cout to outfile
    std::streambuf *backup = std::cout.rdbuf();
    std::ofstream of(outfile_name);
    std::cout.rdbuf(of.rdbuf());
    clustersToCSV(cloud_xyz, cluster);
    std::cout.rdbuf(backup);
    of.close();
  } else if (x11) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    colorByCluster(cloud_xyz, cloud_xyzrgb, cluster, cluster_rgb);
    // Visualization
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setPosition(0, 0);
    viewer.setSize(800, 600);
    viewer.setShowFPS(true);
    viewer.setBackgroundColor(1.0, 1.0, 1.0);
    viewer.setCameraPosition(0.0, 0.0, 2000.0, 0.0, 1.0, 0.0);

    if (opt_verbose > 2) {
      visualizeTriplets(viewer, cloud_xyzti_smooth, triplets);
    } else {
      visualizeClusterAsMst(viewer, cloud_xyzti, cluster);
    }
    viewer.addPointCloud(cloud_xyzrgb, "cloud");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
    if (opt_verbose > 2) {
      viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "cloud");
    }

    viewer.spin();
  } else {
    clustersToCSV(cloud_xyz, cluster);
  }
  return 0;
}
