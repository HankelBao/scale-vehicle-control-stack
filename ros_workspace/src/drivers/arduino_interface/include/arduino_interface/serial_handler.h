// ROS include
#include <ros/ros.h>

// Messages
#include <common_msgs/Control.h>

// External packages
#include <serial/serial.h>

class SerialHandler {
private:
  serial::Serial serial_;

  ros::Subscriber sub_;

  enum AckType : uint8_t { OK, NOT_OK };

  enum ControlType : uint8_t { STEERING, THROTTLE };

  struct ControlMessage {
    int8_t throttle = 0;
    int8_t steering = 0;
    uint8_t padding = 0;
  } message_;

public:
  SerialHandler(ros::NodeHandle &n);
  ~SerialHandler();

  void spin();

private:
  void initSerial();

  void establishConnection();

  void sendControls();

  void controlsCallback(const common_msgs::Control::ConstPtr &msg);
};
