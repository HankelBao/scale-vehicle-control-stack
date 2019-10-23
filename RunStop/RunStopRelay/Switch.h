#pragma once

#include "Constants.h"

class Switch {
private:
  const byte pin;

  bool state;

public:
  Switch(const byte pin) : pin(pin), state(OFF) {
    pinMode(pin, OUTPUT);
    off();
  }

  void on() {
    digitalWrite(pin, HIGH);
    state = ON;
  }

  void off() {
    digitalWrite(pin, LOW);
    state = OFF;
  }

  bool getState() { return state; }

  void flip() { state ? off() : on(); }
};
