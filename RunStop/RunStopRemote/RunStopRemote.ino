#include "Constants.h"
#include "LED.h"
#include "Switch.h"
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>

LED statusLED(LED_STATUS_PIN);
Switch powerSwitch(POWER_PIN);
Switch autoSwitch(AUTO_SWITCH_PIN);

IPAddress ip(192, 168, 0, 80); // IP address of the server

const char *ssid = "AutonomousRC"; // SSID of your home WiFi
const char *pass = "AutonomousRC"; // password of your home WiFi
WiFiClient client;

// A name and a password for the OTA service
const char *OTAName = "AutonomousRC";
const char *OTAPassword = "";

enum Command { POWER, AUTO };

struct Message {
  Command command;
};

void setup() {
  Serial.begin(BAUD_RATE);

  startWiFi();
  startOTA();
}

void loop() {
  if (powerSwitch.isActive()) {
    powerSwitch.getState() ? powerSwitch.setState(OFF)
                           : powerSwitch.setState(ON);
    Serial.println("Power Button Pressed");
    Serial.print("Power Button State :: ");
    Serial.println(powerSwitch.getState());
    Serial.print("Power Message Command :: ");
    Serial.println(POWER);
    Message message{POWER};
    send(message);
  }

  if (powerSwitch.getState()) {
    if (autoSwitch.isActive()) {
      autoSwitch.getState() ? autoSwitch.setState(OFF)
                            : autoSwitch.setState(ON);
      Serial.println("Auto Button Pressed");
      Serial.print("Auto Button State :: ");
      Serial.println(autoSwitch.getState());
      Serial.print("Auto Message Command :: ");
      Serial.println(AUTO);
      Message message{AUTO};
      send(message);
    }
  }
}

void send(Message &message) {
  Serial.println("Message Command Sent :: ");
  Serial.println(message.command);
  client.connect(ip, 80);
  short size = sizeof(message);
  byte buffer[size];
  memcpy(buffer, &message, size);
  client.write(buffer, size);
  statusLED.blink();
}

/*__________________________________________________________SETUP_FUNCTIONS__________________________________________________________*/

/**
  Connect to a Wi-Fi access point.
*/
void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.println();
  Serial.println("Connection to the AP");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    statusLED.blink(2, 250);
  }
  statusLED.blink();
}

/**
   Start the OTA service
*/
void startOTA() {
  ArduinoOTA.setHostname(OTAName);
  ArduinoOTA.setPassword(OTAPassword);

  ArduinoOTA.onStart([]() { Serial.println("Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("\r\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready\r\n");
}
