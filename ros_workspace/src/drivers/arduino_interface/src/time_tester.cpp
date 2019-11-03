// ROS include
#include "ros/ros.h"

#include "common_msgs/Control.h"

ros::Time minimum;
ros::Duration adjusted_diff;

void callback(const common_msgs::Control::ConstPtr &msg) {
  if (msg->header.stamp < minimum) {
    minimum = msg->header.stamp;
    adjusted_diff = ros::Time::now() - minimum;
  }

  ros::Time adjusted_time = ros::Time::now() - adjusted_diff;

  ros::Duration diff = msg->header.stamp - adjusted_time;
  ROS_INFO_STREAM("\nNow :: " << ros::Time::now() << "\nMsg time :: "
                              << msg->header.stamp << "\nDiff :: " << diff.nsec);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arduino_interface_time_tester");
  ros::NodeHandle n;

  minimum = ros::Time::now();

  ros::Subscriber sub(n.subscribe("/control/control", 1, callback));

  ros::spin();
}
