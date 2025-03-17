#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "pins_arduino.h" // Include our custom pins for AXIS board
#define VERSION "1.0.0"

void setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("Starting setup...");
    Serial.print("Version: ");
    Serial.println(VERSION);

    // Initialize WiFi
    WiFi.begin("NotARobot", "Microscope-Wrench-Blueberry"); 
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to WiFi!");
    // Print local IP address
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname("wobbler");
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }
      Serial.println("Start updating " + type);
    });

    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd OTA Update");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();

    // LED indicator setup
    pinMode(LED_BUILTIN, OUTPUT); // BLUE LED 44
    pinMode(43, OUTPUT); // GREEN LED 43
    digitalWrite(43, LOW);
    digitalWrite(LED_BUILTIN, LOW);
      
    Serial.println("Setup complete.");
}

void loop() {
  ArduinoOTA.handle();
  delay(10);
}