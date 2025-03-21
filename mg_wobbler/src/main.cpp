#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <FreeRTOS.h>
#include <SimpleFOC.h>
#include <wifi.h>

#include <atomic>

#include "axis_mqtt_tools.h"    // Include our WiFi header
#include "axis_wifi_manager.h"  // Include our MQTT header
#include "pins_arduino.h"       // Include our custom pins for AXIS board
#define VERSION "1.0.16"        // updated dynamically from python script

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

// motor parameters
int pole_pairs = 11;
float phase_resistance = 0.1088;
float kv = 500;

// Setup the motor and driver
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, kv);
BLDCDriver6PWM driver =
    BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);

// make encoder for simplefoc
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI encoder0(CH0_ENC_CS);

// calibrated sensor object from simplefoc
CalibratedSensor sensor = CalibratedSensor(encoder0);

// global atomic variable for the motor stuff to be set by mqtt
std::atomic<float> last_commanded_target = 0;
std::atomic<float> command_vel_p_gain = 0;
std::atomic<float> command_vel_i_gain = 0;
std::atomic<float> command_vel_d_gain = 0;
std::atomic<float> command_vel_lpf = 0;
std::atomic<float> command_pos_p_gain = 0;
std::atomic<float> command_pos_i_gain = 0;
std::atomic<float> command_pos_d_gain = 0;
std::atomic<float> command_pos_lpf = 0;

std::atomic<bool> enable_flag = false;
std::atomic<bool> disable_flag = false;
std::atomic<bool> motors_enabled = false;

// 0 for torque, 1 for velocity, 2 for position
std::atomic<uint> last_commanded_mode = 0;

// make a separate thread for the OTA
TaskHandle_t loop_foc_task;
// make a separate thread for the MQTT publishing
TaskHandle_t mqtt_publish_task;
const int interval_ms = 500;
void mqtt_publish_thread(void *pvParameters)
{
  while (1)
  {
    static unsigned long lastMsg = millis();
    // Handle MQTT connection
    if (!isMQTTConnected())
    {                   // Use our MQTT connection check function
      reconnectMQTT();  // Use our MQTT reconnect function
    }
    mqttLoop();  // Handle MQTT client loop (IMPORTANT)

    // Publish data periodically
    if (millis() - lastMsg > interval_ms)
    {
      // Create JSON document to send data in
      StaticJsonDocument<512> doc;

      // print target of foc
      doc["target"] = motor.target;
      // print the encoder position
      doc["pos"] = motor.shaft_angle;
      // print the encoder velocity
      doc["vel"] = motor.shaft_velocity;

      // print the gains
      doc["vel_p"] = motor.PID_velocity.P;
      doc["vel_i"] = motor.PID_velocity.I;
      doc["vel_d"] = motor.PID_velocity.D;
      doc["vel_lpf"] = motor.LPF_velocity.Tf;
      doc["pos_p"] = motor.P_angle.P;
      doc["pos_i"] = motor.P_angle.I;
      doc["pos_d"] = motor.P_angle.D;
      doc["pos_lpf"] = motor.LPF_angle.Tf;

      // Serialize JSON to string
      char buffer[512];
      serializeJson(doc, buffer, sizeof(buffer));

      // Publish the message
      publishMQTT(buffer);  // Use our MQTT publish function
    }
    vTaskDelay(interval_ms / portTICK_PERIOD_MS);
  }
}

void loop_foc_thread(void *pvParameters)
{
  while (1)
  {
    // Service flags
    if (enable_flag)
    {
      //   Serial.println("Motors are enabled");
      motor.enable();
      enable_flag.store(false);
      motors_enabled.store(true);
    }
    else if (disable_flag)
    {
      //   Serial.println("Motors are disabled");
      motor.disable();
      disable_flag.store(false);
      motors_enabled.store(false);
    }

    // loop simplefoc
    motor.move(last_commanded_target.load());
    motor.loopFOC();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting setup...");
  Serial.print("Version: ");
  Serial.println(VERSION);

  // Initialize WiFi
  setupWiFi();  // Call our WiFi setup function

  ArduinoOTA.setHostname("wobbler");
  ArduinoOTA.onStart(
      []()
      {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        {
          type = "sketch";
        }
        else
        {  // U_SPIFFS
          type = "filesystem";
        }
        Serial.println("Start updating " + type);
      });

  ArduinoOTA.onStart(
      []()
      {
        // Stop motors on OTA
        disable_flag.store(true);

        // Wait to make sure they stop
        delay(100);
      });

  ArduinoOTA.onEnd([]() { Serial.println("\nEnd OTA Update"); });

  ArduinoOTA.onProgress(
      [](unsigned int progress, unsigned int total)
      { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError(
      [](ota_error_t error)
      {
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

  // LED indicator setup
  pinMode(LED_BUILTIN, OUTPUT);  // BLUE LED 44
  pinMode(43, OUTPUT);           // GREEN LED 43
  digitalWrite(43, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
  delay(1000);
  // initialize encoder
  encoder0.init(&hspi);
  // calibrated sensor
  motor.linkSensor(&sensor);

  // motor driver setup
  driver.voltage_power_supply = 8;
  driver.voltage_limit = 8;
  driver.init();

  // link motor to driver and set up
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 0.5;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;

  // set pid values for velocity controller
  motor.PID_velocity.P = 0.01;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1000;
  motor.PID_velocity.limit = 5000;
  motor.LPF_velocity.Tf = 0.01;
  motor.P_angle.P = 10;
  motor.controller = MotionControlType::velocity;

  motor.init();

  // align sensor and start FOC
  sensor.voltage_calibration = 0.5;
  sensor.calibrate(motor);
  motor.linkSensor(&sensor);

  motor.initFOC();

  setupMQTT();                                 // Call our MQTT setup function
  xTaskCreatePinnedToCore(mqtt_publish_thread, /* Task function. */
                          "MQTT_Publish",      /* String with name of task. */
                          10000,               /* Stack size in bytes. */
                          NULL, /* Parameter passed as input of the task */
                          1,    /* Priority of the task. */
                          &mqtt_publish_task, /* Task handle. */
                          1); /* Core 1 because wifi runs on core 0 */

  // task for arduinoOTA
  xTaskCreatePinnedToCore(loop_foc_thread, "loop_foc", 10000, NULL, 1,
                          &loop_foc_task, 1);

  Serial.println("Setup complete.");
}

void loop()
{
  // if commands have changed, disable the motor, update the values, and
  // re-enable the motor
  if (last_commanded_mode.load() != motor.controller)
  {
    disable_flag.store(true);
    delay(1);
    while (motors_enabled.load())
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    uint controlmode = (MotionControlType)last_commanded_mode.load();
    switch (controlmode)
    {
      case 0:
        motor.controller = MotionControlType::torque;
        break;
      case 1:
        motor.controller = MotionControlType::velocity;
        break;
      case 2:
        motor.controller = MotionControlType::velocity_openloop;
        break;
      default:
        motor.controller = MotionControlType::torque;
        break;
    }
    enable_flag.store(true);
  }

  //   if the gains have changed, disable the motor, update the values, and
  //   re-enable the motor
  if ((command_vel_p_gain.load() != motor.PID_velocity.P) ||
      (command_vel_i_gain.load() != motor.PID_velocity.I) ||
      (command_vel_d_gain.load() != motor.PID_velocity.D) ||
      (command_vel_lpf.load() != motor.LPF_velocity.Tf) ||
      (command_pos_p_gain.load() != motor.P_angle.P) ||
      (command_pos_i_gain.load() != motor.P_angle.I) ||
      (command_pos_d_gain.load() != motor.P_angle.D) ||
      (command_pos_lpf.load() != motor.LPF_angle.Tf))
  {
    disable_flag.store(true);
    delay(1);
    while (motors_enabled.load())
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    motor.PID_velocity.P = command_vel_p_gain.load();
    motor.PID_velocity.I = command_vel_i_gain.load();
    motor.PID_velocity.D = command_vel_d_gain.load();
    motor.LPF_velocity.Tf = command_vel_lpf.load();
    motor.P_angle.P = command_pos_p_gain.load();
    motor.P_angle.I = command_pos_i_gain.load();
    motor.P_angle.D = command_pos_d_gain.load();
    motor.LPF_angle.Tf = command_pos_lpf.load();
    enable_flag.store(true);
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);

  //   Handle OTA updates
  ArduinoOTA.handle();
}