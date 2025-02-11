#include "axis_mqtt_tools.h"
#include <WiFi.h> // Needed for WiFi.macAddress()
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <atomic>

// --- MQTT Broker Settings ---
const char* mqtt_server = "192.168.5.100";
const int mqtt_port = 1883;
const char* mqtt_user = "motorgo"; 
const char* mqtt_password = "axis"; 
const char* mqtt_topic = "axis_data";

// subscribe topic
const char* mqtt_sub_topic = "Axis_cmd";
extern std::atomic<float> last_commanded_target;

WiFiClient espClient;
PubSubClient client(espClient);  // Define the client object

void setupMQTT() {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqtt_callback);
    client.setBufferSize(512);
}

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "MotorGoAxis-";
        clientId += String(WiFi.macAddress());
        Serial.print("Client ID: ");
        Serial.println(clientId);

        if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("MQTT connected");
            if (client.subscribe("Axis_cmd")) {
                Serial.println("Subscribed to Axis_cmd");
            } else {
                Serial.print("Subscribe failed! MQTT State: ");
                Serial.println(client.state());
            }
        } else {
            Serial.print("MQTT connection failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void mqttLoop() {
    client.loop();
}

void publishMQTT(const char* payload) {
    if (client.publish(mqtt_topic, payload)) {
        // Serial.print("Published: ");
        // Serial.println(payload);
    } else {
        Serial.print("Publish failed! MQTT State: ");
        Serial.println(client.state());
    }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming messages
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  // Create a null-terminated string from the payload
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0'; // Null-terminate
  Serial.println(message);

  // Handle subscribed topics
  if (strcmp(topic, mqtt_sub_topic) == 0) {
    // Parse the incoming message
    StaticJsonDocument<512> doc; // Adjust size as needed
    DeserializationError error = deserializeJson(doc, message);

    // Check for parsing errors
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return; // Important:  Exit if parsing fails
    }

    // --- Corrected Key Access ---
    if (doc.containsKey("target_velocity")) { // Check if the key exists
        float target = doc["target_velocity"].as<float>(); // Use the correct key and .as<float>()

        Serial.print("Received target setting commanded from: ");
        Serial.print(last_commanded_target.load()); // Use .load() for atomic
        Serial.print(" to: ");
        Serial.println(target);

        last_commanded_target.store(target); // Atomic store
    } else {
        Serial.println("Invalid message format: missing 'target_velocity' key");
    }
  }
}

bool isMQTTConnected() {
    return client.connected();
}