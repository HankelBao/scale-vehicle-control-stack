#pragma once

class LED {
private:
  const byte pin;

public:
  LED(const byte pin) : pin(pin) { pinMode(pin, OUTPUT); }

  void on() { digitalWrite(pin, HIGH); }

  void off() { digitalWrite(pin, LOW); }

  void blink(const short count = 10, const short ms = 100) {
    for (int i = 0; i < count; i++) {
      on();
      delay(ms);
      off();
      delay(ms);
    }
  }
};
