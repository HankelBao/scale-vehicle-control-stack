// ROS include
#include "ros/ros.h"
#include "point_cloud_thresholder/cone_detector.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "point_cloud_thresholder");
  ros::NodeHandle n("~");

  PointCloudConeDetector cone_detector(n);

  ros::spin();
}
