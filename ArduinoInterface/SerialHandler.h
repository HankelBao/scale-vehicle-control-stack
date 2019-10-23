#pragma once

#include <HardwareSerial.h>

class SerialHandler {
private:
  // Public class variables
  struct ControlMessage {
    int8_t throttle;
    int8_t steering;
    uint8_t padding = 0;
  } message_;

public:
  // Public methods
  SerialHandler() {}

  void Advance() {
    while (Serial.available() > 0) {
      // read the incoming byte:
      uint8_t sizeBuffer[sizeof(uint8_t)];
      Serial.readBytes(sizeBuffer, sizeof(uint8_t));
      uint8_t size = ((uint8_t *)sizeBuffer)[0];
      uint8_t messageBuffer[size];
      Serial.readBytes(messageBuffer, size);
      message_ = *((struct ControlMessage *)messageBuffer);
      Serial.println();
    }
  }

  void establishConnection() {
    while (Serial.available() <= 0) {
      Serial.println();
      delay(300);
    }
  }

  int8_t GetThrottle() { return message_.throttle; }

  int8_t GetSteering() { return message_.steering; }
};
