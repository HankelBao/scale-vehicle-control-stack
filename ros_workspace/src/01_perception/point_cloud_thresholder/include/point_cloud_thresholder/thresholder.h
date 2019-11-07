#pragma once

#include "ros/ros.h"

#include "common_utilities/Color.h"
#include "pcl/PCLPointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include "perception_msgs/ConeXY.h"

using namespace common_utilities;

class PointCloudThresholder {
private:
  pcl::PCLPointCloud2 thresholded_msg_;

public:
  PointCloudThresholder();

  void Threshold(pcl::PCLPointCloud2::Ptr &thresholded_msg,
                 const pcl::PCLPointCloud2::ConstPtr &msg);

private:
  void createPointCloud2(pcl::PCLPointCloud2::Ptr &thresholded_msg);
};
