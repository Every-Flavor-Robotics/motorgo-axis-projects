#ifndef MQTTCONNECTION_H
#define MQTTCONNECTION_H

#include <PubSubClient.h> // Make sure this is installed

extern PubSubClient client; // Declare the client object as extern

void setupMQTT();
void reconnectMQTT();
void mqttLoop();
void publishMQTT(const char* payload);
void mqtt_callback(char* topic, byte* payload, unsigned int length); // Callback function
bool isMQTTConnected();

#endif