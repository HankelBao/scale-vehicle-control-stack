#include "PWMController.h"
#include "SerialHandler.h"

//  --- Constants --- //
const long BAUD_RATE = 115200;
const int UPDATE_RATE = 1; // Time between controller updates

// --- Steering --- //
const uint8_t STEERING_PIN = 9; // Pin on arduino

// --- Motor --- //
const uint8_t MOTOR_PIN = 6; // Pin on arduino

// --- Timer --- //
int time = 0;

// Custom objects
SerialHandler serialHandler;
PWMController motorController;
PWMController steeringController;

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(20);

  motorController.Initialize(PWMController::MOTOR_REVERSE, MOTOR_PIN);
  steeringController.Initialize(PWMController::STEERING_REVERSE, STEERING_PIN);

  serialHandler.establishConnection();
}

void loop() {
  serialHandler.Advance();

  int temp_time = millis();
  if (temp_time - time >= UPDATE_RATE) {
    motorController.Advance(serialHandler.GetThrottle());

    steeringController.Advance(serialHandler.GetSteering());
    time = temp_time;
  }
}
