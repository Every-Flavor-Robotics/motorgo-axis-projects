#include "axis_mqtt_tools.h"
#include <WiFi.h> // Needed for WiFi.macAddress()
#include <ArduinoJson.h>

// --- Define the MQTT Broker Settings ---
const char* mqtt_server = "192.168.10.60"; //  <<--- YOUR MQTT BROKER IP
const int mqtt_port = 1883;
const char* mqtt_user = "motorgo";
const char* mqtt_password = "axis";
const char* mqtt_topic = "wobbler";         // General data topic
const char* mqtt_sub_topic = "wobbler/cmd/#"; // Subscribe topic
const char* mqtt_ota_topic = "wobbler/ota/update";        // OTA topic
const char* mqtt_ota_status_topic = "wobbler/ota/status"; // OTA Status

WiFiClient espClient;
PubSubClient client(espClient);  // Define the client object

// Define the atomic variables (allocate memory for them)
extern std::atomic<uint8_t> mqtt_update_freq_hz;
extern std::atomic<float> com_motor_torque;
extern std::atomic<float> com_balance_pt_rad;
extern std::atomic<float> com_balance_offset_volts;
extern std::atomic<float> com_bal_p_gain;
extern std::atomic<float> com_bal_i_gain;
extern std::atomic<float> com_bal_d_gain;
extern std::atomic<float> com_vel_lpf;
extern std::atomic<bool> com_x_dir;
extern std::atomic<bool> com_y_dir;
extern std::atomic<float> com_balance_pt_p_gain;
extern std::atomic<float> com_balance_pt_i_gain;
extern std::atomic<float> com_balance_pt_d_gain;
extern std::atomic<bool> enable_flag;
extern std::atomic<bool> disable_flag;
extern std::atomic<bool> motors_enabled;
extern std::atomic<uint8_t> com_mode;
extern std::atomic<bool> update_pid_flag;

void setupMQTT() {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqtt_callback);
    client.setBufferSize(512);
}

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "Wobbler-";
        clientId += String(WiFi.macAddress());
        Serial.print("Client ID: ");
        Serial.println(clientId);

        if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("MQTT connected");
            client.subscribe(mqtt_ota_topic);    // Subscribe to OTA
            client.subscribe(mqtt_sub_topic);   // Subscribe to commands
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
    publishMQTT(payload, mqtt_topic); // Call the overloaded version
}

void publishMQTT(const char* payload, const char* topic) {
    if (client.publish(topic, payload)) {
        Serial.print("Published message to topic: ");
        Serial.println(topic);
    } else {
        Serial.println("Failed to publish message");
    }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");

    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.println(message);

    if (strcmp(topic, mqtt_ota_topic) == 0) {
        if (strcmp(message, "update") == 0) {
            Serial.println("OTA update requested");
            publishMQTT("Starting OTA update", mqtt_ota_status_topic);
        }
    } 
    // else if topic string contains "cmd/"
    else if (strstr(topic, "cmd/") != NULL) {
        StaticJsonDocument<512> doc;
        deserializeJson(doc, message);
        
        if (doc.containsKey("freq")) {
            mqtt_update_freq_hz.store(doc["freq"]);
        }
        if (doc.containsKey("mode")) {
            com_mode.store(doc["mode"]);
        }
        if (doc.containsKey("torque")) {
            com_motor_torque.store(doc["torque"]);
        }
        if (doc.containsKey("balance_pt_rad")) {
            com_balance_pt_rad.store(doc["balance_pt_rad"]);
        }
        if (doc.containsKey("balance_offset_volts")) {
            com_balance_offset_volts.store(doc["balance_offset_volts"]);
        }
        if (doc.containsKey("bal_p_gain")) {
            com_bal_p_gain.store(doc["bal_p_gain"]);
            update_pid_flag.store(true);
        }
        if (doc.containsKey("bal_i_gain")) {
            com_bal_i_gain.store(doc["bal_i_gain"]);
            update_pid_flag.store(true);
        }
        if (doc.containsKey("bal_d_gain")) {
            com_bal_d_gain.store(doc["bal_d_gain"]);
            update_pid_flag.store(true);
        }
        if (doc.containsKey("vel_lpf")) {
            com_vel_lpf.store(doc["vel_lpf"]);
            update_pid_flag.store(true);
        }
        if (doc.containsKey("x_dir")) {
            com_x_dir.store(doc["x_dir"]);
        }
        if (doc.containsKey("y_dir")) {
            com_y_dir.store(doc["y_dir"]);
        }
        if (doc.containsKey("balance_pt_p_gain")) {
            com_balance_pt_p_gain.store(doc["balance_pt_p_gain"]);
            update_pid_flag.store(true);
        }
        if (doc.containsKey("balance_pt_i_gain")) {
            com_balance_pt_i_gain.store(doc["balance_pt_i_gain"]);
            update_pid_flag.store(true);
        }
        if (doc.containsKey("balance_pt_d_gain")) {
            com_balance_pt_d_gain.store(doc["balance_pt_d_gain"]);
            update_pid_flag.store(true);
        }
        if (doc.containsKey("enable")) {
            enable_flag.store(doc["enable"]);
        }
        if (doc.containsKey("disable")) {
            disable_flag.store(doc["disable"]);
        }
        
    }
}

bool isMQTTConnected() {
    return client.connected();
}