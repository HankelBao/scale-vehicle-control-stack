 #pragma once
 
 #include <Servo.h>

 class PWMController {
 public:
  void Setup(char s, int pin, int normal_uS, int min_uS, int max_uS, int max_delta_uS, boolean r) {
    symbol = s;
    reversed = r;
    normal = normal_uS;
    neg_val = normal_uS - min_uS;
    pos_val = max_uS - normal_uS;
    max_delta = max_delta_uS;
    servo.attach(pin);
    Reset();
  }

  void Reset() {
    servo.writeMicroseconds(normal);
    target = normal;
    current = normal;
  }
  
  void SetTarget(int percent) {
    if (percent > 100)
      percent = 100;
    if (percent < -100)
      percent = -100;

    percent = reversed? -percent: percent;
    if (percent>0)
      target = normal + percent/100.00f*pos_val;
    else
      target = normal + percent/100.00f*neg_val;
  }
  
  void Advance() {
    if (current == target)
      return;
      
    int step_target = target;
    step_target += (1/1000.0f)*15*(target-current);
    
    if (step_target > current + max_delta)
      step_target = current + max_delta;
    if (step_target < current - max_delta)
      step_target = current - max_delta;

    servo.writeMicroseconds(step_target);
    current = step_target;
    PrintStatus();
  }

  int CurrentPW() {
    return current;
  }

  int CurrentPercent() {
    int val;
    if (current > normal)
      val = (float)(current-normal)/pos_val*100;
    else
      val = -(float)(normal-current)/neg_val*100;
    return reversed? -val: val;
  }

  void PrintStatus() {
    Serial.print(symbol);
    Serial.println(CurrentPercent());
  }

private:
  char symbol;
  boolean reversed;
  int normal;
  int neg_val;
  int pos_val;
  int max_delta;
  
  int target;
  int current;
  
  Servo servo;
};

