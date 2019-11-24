#include "PWMController.h"
#include "SerialHandler.h"

const int BAUD_RATE = 115200;
const int TIMEOUT = 20;
const int UPDATE_RATE = 1;
const int DELTA_MAX = 20;

const int STEERING_PIN = 9;
const int STEERING_NORMAL_PW = 1520;
const int STEERING_MIN_PW = 1160;
const int STEERING_MAX_PW = 1920;

const int MOTOR_PIN = 6;
const int MOTOR_NORMAL_PW = 1560;
const int MOTOR_MIN_PW = 1040;
const int MOTOR_MAX_PW = 2000;

PWMController steeringController;
PWMController motorController;
SerialHandler serialHandler(BAUD_RATE, TIMEOUT);

int last_loop_time = 0;

void setup() {
  steeringController.Setup('D', STEERING_PIN, STEERING_NORMAL_PW, STEERING_MIN_PW, STEERING_MAX_PW, DELTA_MAX, true);
  motorController.Setup('T', MOTOR_PIN, MOTOR_NORMAL_PW, MOTOR_MIN_PW, MOTOR_MAX_PW, DELTA_MAX, false);
  serialHandler.establishConnection();
  Serial.println("Ready");
}

void loop() {
  if (serialHandler.getCommand()) {
    switch (serialHandler.command.type) {
      case CommandType::DIRECTION:
        steeringController.SetTarget(serialHandler.command.value);
        break;
      case CommandType::THROTTLE:
        motorController.SetTarget(serialHandler.command.value);
        break;
    }
  }

  int current_time = millis();
  if (current_time - last_loop_time >= UPDATE_RATE) {
    steeringController.Advance();
    motorController.Advance();
    last_loop_time = current_time;
  }
}
