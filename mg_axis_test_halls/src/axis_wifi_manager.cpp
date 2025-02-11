#include "axis_wifi_manager.h"
#include <WiFi.h>
#include <WiFiManager.h>

void setupWiFi() {
    WiFiManager wifiManager;
    wifiManager.setConnectTimeout(10);
    wifiManager.setConfigPortalTimeout(60);

    Serial.println("Connecting to WiFi...");
    if (!wifiManager.autoConnect("ESP32-DataLogger", "password")) {
        Serial.println("Failed to connect and hit timeout. Restarting...");
        ESP.restart();
    }

    Serial.println("Connected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
        delay(500);
        Serial.print(".");
        if (millis() - startTime > 15000) {
            Serial.println("\nFailed to establish a stable WiFi connection. Restarting...");
            ESP.restart();
        }
    }
    Serial.println("\nWiFi connection stable.");
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.println("WiFi setup complete.");
}