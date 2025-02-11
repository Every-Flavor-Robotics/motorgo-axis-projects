#include <Arduino.h>
#include <SimpleFOC.h>
#include <WiFi.h> // For WiFi.macAddress()
#include <WiFiManager.h> // For WiFiManager
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <atomic>
#include <motorgo_mini.h>

// --- Configuration (Adjust these for your specific setup) ---

// --- Motor Parameters ---
#define MOTOR_POLE_PAIRS 7 // Replace with your motor's pole pairs

// --- MQTT Broker Settings ---
const char* mqtt_server = "192.168.5.100";  // Replace with your broker's IP
const int mqtt_port = 1883;
const char* mqtt_user = "motorgo";     // Replace with your MQTT username
const char* mqtt_password = "axis";   // Replace with your MQTT password
const char* mqtt_topic = "Axis_cmd/target_velocity"; // Topic to PUBLISH motor position
const char* mqtt_sub_topic = "Mini_cmd";  // Topic to SUBSCRIBE for target torque

// --- Global Variables ---
std::atomic<float> last_commanded_torque(0.0f); // Use atomic for thread safety

// --- SimpleFOC Objects ---
MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& motor_left = motorgo_mini.ch1;
MotorGo::ChannelConfiguration config_left;
MotorGo::PIDParameters velocity_pid_params_left;

// --- WiFi and MQTT Objects ---
WiFiClient espClient;
PubSubClient client(espClient);

// --- Function Prototypes ---
void setupWiFi();
void setupMQTT();
void reconnectMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
bool isMQTTConnected();
void publishMQTT(const char* payload);

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Setup motor parameters
 
  config_left.motor_config = MotorGo::MotorGoGreen;
  config_left.power_supply_voltage = 5.0;
  config_left.reversed = false; 

  bool calibrate = false;
  // Initialize motors  
  motor_left.init(config_left, calibrate);
  motor_left.set_control_mode(MotorGo::ControlMode::Position);

  // Setup
  // --- Initialize WiFi (using WiFiManager) ---
  setupWiFi();
  // --- Initialize MQTT ---
  setupMQTT();

  Serial.println("Setup complete.");
  motor_left.enable(); //moved down here to be the last part of the code

}

// --- Main Loop ---
void loop() {
  // --- Handle WiFi and MQTT ---
    if (!isMQTTConnected()) {
        reconnectMQTT();
    }
    client.loop();

  motor_left.loop();
  
  // --- Create JSON payload and publish to MQTT broker ---
  StaticJsonDocument<256> doc;
  doc["position"] = motor_left.get_position();
  char buffer[256];
  serializeJson(doc, buffer);
  publishMQTT(buffer);

  //delay vtask
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

// --- WiFi Setup (using WiFiManager) ---
void setupWiFi() {
    WiFiManager wifiManager;
    wifiManager.setConnectTimeout(10); // Set connection timeout
    wifiManager.setConfigPortalTimeout(60); // Set config portal timeout

    Serial.println("Connecting to WiFi...");
    if (!wifiManager.autoConnect("ESP32-HapticControl", "password")) { //  Use a descriptive AP name
        Serial.println("Failed to connect and hit timeout. Restarting...");
        ESP.restart();
    }

    Serial.println("Connected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Wait for a *stable* connection (important for MQTT)
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
        delay(500);
        Serial.print(".");
        if (millis() - startTime > 15000) { // 15-second timeout
            Serial.println("\nFailed to establish a stable WiFi connection. Restarting...");
            ESP.restart();
        }
    }

    Serial.println("\nWiFi connection stable.");
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
}
// --- MQTT Setup ---
void setupMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
  client.setBufferSize(512); // Set buffer size (adjust if needed)
}

// --- MQTT Reconnect ---
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(WiFi.macAddress()); // Unique client ID
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(mqtt_sub_topic); // Subscribe to the command topic
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// --- MQTT Callback ---
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  Serial.println(message);

  if (strcmp(topic, mqtt_sub_topic) == 0) {
    StaticJsonDocument<256> doc; //  Make sure this is big enough
    DeserializationError error = deserializeJson(doc, message);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }

    if (doc.containsKey("target_torque")) { // Use the correct key
        float target = doc["target_torque"].as<float>();
        Serial.print("Received target torque: ");
        Serial.println(target);
        last_commanded_torque.store(target);
    }
    else{
         Serial.println("target_torque not found in json doc");
    }
  }
}

// --- MQTT Publish ---
void publishMQTT(const char* payload) {
  if (client.publish(mqtt_topic, payload)) {
      Serial.print("Published: ");
      Serial.println(payload);
  } else {
      Serial.print("Publish failed! MQTT State: ");
      Serial.println(client.state());
 // You might want to call reconnectMQTT() here if publishing fails
  }
}

bool isMQTTConnected() {
     return client.connected();
}

