#include "point_cloud_thresholder/thresholder.h"

PointCloudThresholder::PointCloudThresholder() {}

void PointCloudThresholder::Threshold(
    pcl::PCLPointCloud2::Ptr &thresholded_msg,
    const pcl::PCLPointCloud2::ConstPtr &msg) {
  ColorRange green(Color(20, 120, 0), Color(200, 255, 50));
  ColorRange red(Color(0, 0, 150), Color(100, 100, 255));

  int j = 0;
  int color_offset = 12;
  int offset = 16;
  for (int i = color_offset; i < (msg->row_step * msg->height); i += offset) {

    Color point_color(msg->data[i], msg->data[i + 1], msg->data[i + 2]);

    if (green.WithinRange(point_color)) {
      thresholded_msg->data.insert(thresholded_msg->data.end(),
                                   &msg->data[i - color_offset],
                                   &msg->data[i + 4]);
      thresholded_msg->data.back() = perception_msgs::ConeXY::GREEN;
    } else if (red.WithinRange(point_color)) {
      thresholded_msg->data.insert(thresholded_msg->data.end(),
                                   &msg->data[i - color_offset],
                                   &msg->data[i + 4]);
      thresholded_msg->data.back() = perception_msgs::ConeXY::RED;
    }
  }
  createPointCloud2(thresholded_msg);
}

void PointCloudThresholder::createPointCloud2(
    pcl::PCLPointCloud2::Ptr &thresholded_msg) {
  pcl_conversions::toPCL(ros::Time::now(), thresholded_msg->header.stamp);
  thresholded_msg->header.frame_id = "zed_left_camera_frame";

  // Convert x/y/z to fields
  thresholded_msg->fields.resize(4);
  thresholded_msg->fields[0].name = "x";
  thresholded_msg->fields[1].name = "y";
  thresholded_msg->fields[2].name = "z";
  thresholded_msg->fields[3].name = "rgb";

  int offset = 0;
  for (size_t d = 0; d < thresholded_msg->fields.size(); d++, offset += 4) {
    thresholded_msg->fields[d].offset = offset;
    thresholded_msg->fields[d].datatype = pcl::PCLPointField::FLOAT32;
    thresholded_msg->fields[d].count = 1;
  }

  thresholded_msg->point_step = offset;

  thresholded_msg->is_bigendian = false;
  thresholded_msg->is_dense = false;

  thresholded_msg->width = thresholded_msg->data.size() / 16;
  thresholded_msg->height = 1;

  thresholded_msg->row_step =
      thresholded_msg->point_step * thresholded_msg->width;
}
