#include "point_cloud_thresholder/clusterer.h"

#include <thread>

#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGBA PointType;

PointCloudClusterer::PointCloudClusterer() {}

void PointCloudClusterer::Cluster(perception_msgs::ConeDepthMap &depth_map,
                                  pcl::PCLPointCloud2::Ptr &msg) {
  if (msg->width == 0) {
    return;
  }

  // Convert the pcl/PCLPointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>),
      cloud(new pcl::PointCloud<PointType>),
      cloud_f(new pcl::PointCloud<PointType>);
  pcl::fromPCLPointCloud2(*msg, *cloud);

  // Eigen::Vector3f rotation_vector;
  // rotation_vector << 0, 1, 0;
  //
  // float theta = M_PI / 9;
  //
  // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // transform_2.translation() << 0, 0, 0;
  // transform_2.rotate(Eigen::AngleAxisf(theta, rotation_vector));
  // pcl::transformPointCloud(*cloud_in, *cloud, transform_2);

  // pcl::console::TicToc tt;
  // std::cerr << "Starting...\n", tt.tic();

  // Create the filtering object: downsample the dataset using a leaf size of
  // 1cm
  pcl::VoxelGrid<PointType> vg;
  pcl::PointCloud<PointType>::Ptr cloud_down_sampled(
      new pcl::PointCloud<PointType>),
      cloud_filtered1(new pcl::PointCloud<PointType>),
      cloud_filtered(new pcl::PointCloud<PointType>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  // vg.filter(*cloud_down_sampled);
  vg.filter(*cloud_filtered);
  // std::cout << "PointCloud after filtering has: "
  // << cloud_filtered->points.size() << " data points."
  // << std::endl; //*

  // Create the filtering object
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(cloud_down_sampled);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(.5, 3);
  // pass.setFilterLimitsNegative (true);
  // pass.filter(*cloud_filtered);

  // pcl::RadiusOutlierRemoval<PointType> outrem;
  // build the filter
  // outrem.setInputCloud(cloud_down_sampled);
  // outrem.setRadiusSearch(0.04);
  // outrem.setMinNeighborsInRadius(25);
  // apply filter
  // outrem.filter(*cloud_filtered);

  // Create the filtering object
  // pcl::StatisticalOutlierRemoval<PointType> sor;
  // sor.setInputCloud (cloud_down_sampled);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*cloud_filtered);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointCloud<PointType>::Ptr cloud_plane(new pcl::PointCloud<PointType>());

  int i = 0, nr_points = (int)cloud_filtered->points.size();
  while (cloud_filtered->points.size() > 0.3 * nr_points) {
    if (inliers->indices.size() == 0) {
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the cone
    extract.filter(*cloud_plane);

    // Remove the cone inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(.05); // 2cm
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(10000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  // std::cout << "Length of Point Indices :: " << cluster_indices.size()
  // << std::endl;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<PointType>::Ptr cloud_cluster(
        new pcl::PointCloud<PointType>);
    bool is_green;
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      is_green =
          cloud_filtered->points[*pit].a == perception_msgs::ConeXY::GREEN;
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Find centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    // cout << centroid[0] << " " << centroid[1] << " " << centroid[2] << " "
    // << centroid[3] << " \n";

    perception_msgs::ConeXY cone;
    cone.pos.x = centroid[0];
    cone.pos.y = centroid[1];
    cone.pos.z = centroid[2];
    if (is_green) {
      cone.color = perception_msgs::ConeXY::GREEN;
      depth_map.green_cones.push_back(cone);
    } else {
      cone.color = perception_msgs::ConeXY::RED;
      depth_map.red_cones.push_back(cone);
    }
  }

  // std::cerr << ">> Done: " << tt.toc() << " ms, " << cluster_indices.size()
  // << " clusters found\n";
}
