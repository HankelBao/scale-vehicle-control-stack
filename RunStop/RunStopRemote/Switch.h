#pragma once

#include "Constants.h"

class Switch {
private:
  const byte pin;

  bool state;

public:
  Switch(const byte pin) : pin(pin), state(OFF) { pinMode(pin, INPUT); }

  bool getState() { return state; }

  void setState(bool state) { this->state = state; }

  bool isActive() { return digitalRead(pin); }
};
