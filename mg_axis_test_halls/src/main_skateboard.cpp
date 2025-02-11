#include <Arduino.h>
#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include "axis_mqtt_tools.h"  // Include our WiFi header
#include "axis_wifi_manager.h" // Include our MQTT header
#include "pins_arduino.h" // Include our custom pins for AXIS board

#include <SimpleFOC.h>
#include <atomic>


// motor parameters
int pole_pairs = 10;
// motor hall effect sensor
int hall_a = 37;
int hall_b = 39;
int hall_c = 40;
HallSensor halls = HallSensor(hall_a, hall_b, hall_c, pole_pairs);

// make a motor instance in simplefoc for the open loop test
BLDCMotor motor = BLDCMotor(pole_pairs);
BLDCDriver6PWM driver = BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);


// global atomic variable for the motor target to be set by mqtt
std::atomic<float> last_commanded_target = 0;

//make a separate thread for the MQTT publishing
TaskHandle_t mqtt_publish_task;
const int interval_ms = 100;
void mqtt_publish_thread(void *pvParameters) {
    while (1) {
      static unsigned long lastMsg = millis();
        // Handle MQTT connection
        if (!isMQTTConnected()) {  // Use our MQTT connection check function
            reconnectMQTT();    // Use our MQTT reconnect function
        }
        mqttLoop(); // Handle MQTT client loop (IMPORTANT)

        // Publish data periodically
        if (millis() - lastMsg > interval_ms) {

            // Create JSON document to send data in
            StaticJsonDocument<512> doc;
            // digital read the three halls sensors
            doc["hall_a"] = digitalRead(hall_a);
            doc["hall_b"] = digitalRead(hall_b);
            doc["hall_c"] = digitalRead(hall_c);
            // also add the angle from the halls
            halls.update();
            doc["angle"] = halls.getAngle();

            //print target of foc
            doc["target"] = motor.target;

            // Serialize JSON to string
            char buffer[512];
            serializeJson(doc, buffer, sizeof(buffer));

            // Publish the message
            publishMQTT(buffer); // Use our MQTT publish function
          }
          vTaskDelay(interval_ms / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting setup...");

    // Initialize WiFi
    setupWiFi(); // Call our WiFi setup function

    // Initialize MQTT
    setupMQTT(); // Call our MQTT setup function
    xTaskCreatePinnedToCore(
      mqtt_publish_thread,   /* Task function. */
      "MQTT_Publish",       /* String with name of task. */
      10000,                /* Stack size in bytes. */
      NULL,                 /* Parameter passed as input of the task */
      1,                    /* Priority of the task. */
      &mqtt_publish_task,   /* Task handle. */
      1);                   /* Core 1 because wifi runs on core 0 */
      
      // LED indicator setup
      pinMode(LED_BUILTIN, OUTPUT);
      pinMode(43, OUTPUT);
      digitalWrite(43, LOW);
      
    // hall sensor setup
    halls.pullup = Pullup::USE_INTERN;
    // setup the 3 halls pins to be read by digital read
    pinMode(hall_a, INPUT_PULLUP);
    pinMode(hall_b, INPUT_PULLUP);
    pinMode(hall_c, INPUT_PULLUP);
    halls.init();
    motor.linkSensor(&halls);

    // motor driver setup
    driver.voltage_power_supply = 16;
    driver.voltage_limit = 5;
    driver.init();

    // link motor to driver and set up
    motor.linkDriver(&driver);
    motor.voltage_sensor_align = 1;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::voltage;

    // set pid values for velocity controller
    motor.PID_velocity.P = 0.01;
    motor.PID_velocity.I = 1;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 1000;
    motor.PID_velocity.limit = 50;
    motor.LPF_velocity.Tf = 0.01;

    motor.P_angle.P = 10;

    motor.controller = MotionControlType::angle;
    
    motor.init();
    motor.initFOC();

    Serial.println("Setup complete.");
}

void loop() {
    halls.update();

    //loop simplefoc
    motor.move(last_commanded_target.load());
    motor.loopFOC();
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
}