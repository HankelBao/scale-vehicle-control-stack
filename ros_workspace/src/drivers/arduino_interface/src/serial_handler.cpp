#include "arduino_interface/serial_handler.h"

#include <chrono>
#include <thread>

SerialHandler::SerialHandler(ros::NodeHandle &n)
    : controls_sub_(n.subscribe("/control/control", 1,
                                &SerialHandler::controlsCallback, this)) {
  if (thread_.joinable())
    thread_.join();
}

SerialHandler::~SerialHandler() {
  serial_.close();
  if (thread_.joinable())
    thread_.join();
}

int SerialHandler::initSerial() {
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

void SerialHandler::establishConnection() {
  while (serial_.available() <= 0) {
    ros::Duration(1).sleep();
  }
  serial_.readline();
  serial_.flush();
  serial_.write("1");
  std::cout << "Connection Established" << std::endl;
  sendControls();
}

void SerialHandler::sendControls() {
  uint8_t size = sizeof(message_);
  uint8_t buffer[size + sizeof(uint8_t)];
  memcpy(buffer + sizeof(uint8_t), &message_, size);
  buffer[0] = size;
  serial_.flush();
  serial_.write(buffer, size);
  // std::cout
  //     << "Motor Sent :: "
  //     << (int)(*(struct ControlMessage *)(buffer + sizeof(uint8_t))).throttle
  //     << std::endl;
  // std::cout
  //     << "Steering Sent :: "
  //     << (int)(*(struct ControlMessage *)(buffer + sizeof(uint8_t))).steering
  //     << std::endl;

  serial_.readline();
}

void SerialHandler::controlsCallback(
    const common_msgs::Control::ConstPtr &msg) {
  auto start = std::chrono::high_resolution_clock::now();

  // ros::Duration diff = ros::Time::now() - msg->header.stamp;
  // ROS_INFO_STREAM("Diff :: " << diff);

  // Get controls
  int8_t throttle = msg->throttle * 100;
  int8_t steering = msg->steering * 100;

  // Clamp controls
  int max_throttle = 15;
  throttle = throttle > max_throttle ? max_throttle : throttle;
  //  if (abs(steering * 1.5) < 100)
  // steering *= 1.5;
  steering -= 5;

  message_ = ControlMessage{throttle, steering};

  // if (thread_.joinable())
    // thread_.join();

  // std::thread thr([&] {
    // mutex_.lock();
    // sendControls();
    // mutex_.unlock();
  // });
  // thread_ = std::move(thr);

  sendControls();
  auto end = std::chrono::high_resolution_clock::now();

  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  ROS_DEBUG_STREAM("INTERFACE :: " << (duration.count() * 1e-3)
                                  << " milliseconds");
}

int SerialHandler::spin() {
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
    ros::spinOnce();
  }
  message_ = ControlMessage{0, 0};
  sendControls();
  return 0;
}
