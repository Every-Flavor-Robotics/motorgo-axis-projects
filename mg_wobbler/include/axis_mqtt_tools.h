#ifndef AXIS_MQTT_TOOLS_H
#define AXIS_MQTT_TOOLS_H

#include <PubSubClient.h> // Make sure this is installed
#include <atomic> // Include for std::atomic

// Declare the client object as extern (it's defined in the .cpp)
extern PubSubClient client;

// Declare MQTT settings as extern const char*
extern const char* mqtt_server;
extern const int mqtt_port;
extern const char* mqtt_user;
extern const char* mqtt_password;
extern const char* mqtt_topic; // General purpose topic
extern const char* mqtt_sub_topic; // Subscribe topic
extern const char* mqtt_ota_topic;
extern const char* mqtt_ota_status_topic;

// Declare the atomic variables as extern
extern std::atomic<float> last_commanded_target;
extern std::atomic<uint> last_commanded_mode;

void setupMQTT();
void reconnectMQTT();
void mqttLoop();
void publishMQTT(const char* payload); // Original publish function
void publishMQTT(const char* payload, const char* topic); // overloaded publish function
void mqtt_callback(char* topic, byte* payload, unsigned int length); // Callback function
bool isMQTTConnected();

#endif