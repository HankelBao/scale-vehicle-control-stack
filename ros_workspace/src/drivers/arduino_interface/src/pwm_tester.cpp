// ROS include
#include "ros/ros.h"

// Messages
#include "common_msgs/Control.h"

// External packages
#include "serial/serial.h"

class SerialHandler {
private:
  serial::Serial serial_;

  struct ControlMessage {
    int8_t throttle = 0;
    int8_t steering = 0;
    uint8_t padding = 0;
  } message_;

public:
  SerialHandler() {}

  ~SerialHandler() { serial_.close(); }

  int initSerial() {
    try {
      serial_.setPort("/dev/ttyACM0");
      serial_.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      serial_.setTimeout(to);
      serial_.open();
    } catch (serial::IOException &e) {
      std::cout << "Serial Port Initialization Unsuccessful" << std::endl;
      serial_.close();
      ros::shutdown();
      return -1;
    }

    if (serial_.isOpen()) {
      std::cout << "Serial Port Initializated" << std::endl;
    } else {
      serial_.close();
      ros::shutdown();
      return -1;
    }
  }

  void establishConnection() {
    while (serial_.available() <= 0) {
      ros::Duration(1).sleep();
    }
    serial_.readline();
    serial_.flush();
    serial_.write("1");
    std::cout << "Connection Established" << std::endl;
    sendControls(0);
  }

  void sendControls(int steering) {
    message_ = ControlMessage{0, (int8_t)steering};
    uint8_t size = sizeof(message_);
    uint8_t buffer[size + sizeof(uint8_t)];
    memcpy(buffer + sizeof(uint8_t), &message_, size);
    buffer[0] = size;
    serial_.flush();
    serial_.write(buffer, size);
    std::cout
        << "Motor Sent :: "
        << (int)(*(struct ControlMessage *)(buffer + sizeof(uint8_t))).throttle
        << std::endl;
    std::cout
        << "Steering Sent :: "
        << (int)(*(struct ControlMessage *)(buffer + sizeof(uint8_t))).steering
        << std::endl;

    serial_.readline();
  }

  int spin() {
    if (initSerial() == -1)
      return -1;
    ros::Duration(2).sleep();
    establishConnection();

    while (ros::ok()) {
      if (serial_.available()) {
        // serial_.flush();
        std::string msg = serial_.readline();
        std::cout << "Data Received :: " << msg << std::endl;
      }
      int steering;
      std::cin >> steering;
      sendControls(steering);
      ros::Duration(0.5).sleep();
    }
    sendControls(0);
    return 0;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "arduino_interface_pwm_tester");
  ros::NodeHandle n;

  SerialHandler serialHandler;

  return serialHandler.spin();
}
