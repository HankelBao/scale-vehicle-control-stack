#pragma once

#include "ros/ros.h"

#include "pcl/PCLPointCloud2.h"
#include "perception_msgs/ConeDepthMap.h"

class PointCloudClusterer {
public:
  PointCloudClusterer();

  void Cluster(perception_msgs::ConeDepthMap &depth_map,
               pcl::PCLPointCloud2::Ptr &msg);
};
