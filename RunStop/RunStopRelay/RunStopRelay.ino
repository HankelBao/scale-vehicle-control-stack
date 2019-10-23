/**
 * TODO:
 * 1. Emergency stop if/when a client becomes disconnected
 * 2. Create LED system that actually indicates connection or meaning
 */

#include "Constants.h"
#include "LED.h"
#include "Switch.h"
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>

LED statusLED(LED_STATUS_PIN);
Switch powerSwitch(POWER_SWITCH_PIN);
Switch steeringSwitch(STEERING_SWITCH_PIN);
Switch motorSwitch(MOTOR_SWITCH_PIN);

const char *ssid = "AutonomousRC"; // SSID
const char *pass = "AutonomousRC"; // password

IPAddress ip(192, 168, 0, 80);     // IP address of the server
IPAddress gateway(192, 168, 0, 1); // gateway of your network
IPAddress mask(255, 255, 255, 0);  // subnet mask of your network
WiFiServer server(80);

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
  startServer();
  startOTA();
}

void loop() {
  //  if (!server.hasClient()) emergencyStop();
  WiFiClient client = server.available();
  if (!client)
    return;
  while (!client.available()) {
  }
  Message message;
  byte *messageBuffer = (byte *)&message;
  while (client.available()) {
    messageBuffer += client.read(messageBuffer, 1);
  }
  Serial.print("Message Command Received :: ");
  Serial.println(message.command);
  handleMessage(message);
  statusLED.blink();
}

// void emergencyStop() {
//  Serial.println("EMERGENCY STOP");
//  powerSwitch.off();
//  steeringSwitch.off();
//  motorSwitch.off();
//}

/**
  Start a Wi-Fi access point. Wait for the AP connection.
*/
void startWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pass); // Start the access point
  WiFi.softAPConfig(ip, gateway, mask);
  Serial.print("Access Point \"");
  Serial.print(ssid);
  Serial.println("\" started\r\n");
  statusLED.blink();
}

/**
   State the WiFi Server
*/
void startServer() { server.begin(); }

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

void handleMessage(const Message &message) {
  Serial.println("Received Message");
  switch (message.command) {
  case POWER:
    Serial.println("Flipping Power Switch");
    powerSwitch.flip();
    break;
  case AUTO:
    Serial.println("Flipping Auto Switch");
    steeringSwitch.flip();
    motorSwitch.flip();
    break;
  }
}
