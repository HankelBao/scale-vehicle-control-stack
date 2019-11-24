 #pragma once

 enum CommandType : int {
   DIRECTION, THROTTLE
 };

 struct Command {
  CommandType type;
  int value;
 };

 class SerialHandler {
 public:
  Command command;
  
  SerialHandler(int baud_rate, int timeout) {
    BAUD_RATE = baud_rate;
    TIMEOUT = timeout;
    commandString = "";
  }

  void establishConnection() {
    Serial.begin(BAUD_RATE);
    Serial.setTimeout(TIMEOUT);
  }

  boolean getCommand() {
    if (Serial.available() > 0) {
      int inChar = Serial.read();
      switch (inChar) {
      case 'D':
        command.type = CommandType::DIRECTION;
        break;
      case 'T':
        command.type = CommandType::THROTTLE;
        break;
      }
      if (isDigit(inChar) || inChar == '-') {
        commandString += (char)inChar;
      }
      
      if (inChar == '\n') {
        command.value = commandString.toInt();
        
        commandString = "";
        return true;
      }
    }
    return false;
  }

 private:
  String commandString;
  int BAUD_RATE;
  int TIMEOUT;
 };

