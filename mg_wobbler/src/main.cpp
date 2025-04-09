#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <FreeRTOS.h>
#include <SimpleFOC.h>
#include <wifi.h>

#include <atomic>

#include "axis_mqtt_tools.h"   // Include our WiFi header
#include "axis_wifi_manager.h" // Include our MQTT header
#include "imu.h"
#include "pins_arduino.h" // Include our custom pins for AXIS board
#define VERSION "1.0.145"  // updated dynamically from python script

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

// IMU
Imu::Imu imu(0.2);

// atomic variable to adjust mqtt update frequency
std::atomic<uint8_t> mqtt_update_freq_hz = 10; // default to 100ms (10Hz)

// global atomic variable for the motor stuff to be set by mqtt
// target will not be set by us except for debugging
std::atomic<float> com_motor_torque = 0;
std::atomic<float> com_balance_pt_rad = 0; // balance point in radians
std::atomic<float> com_balance_offset_rad = 0; // offset in radians
std::atomic<float> last_balance_target_volts = 0;
std::atomic<float> last_offset_rad = 0;

std::atomic<float> com_feedforward = 0; // to adjust for non-linearity

// gains for the inner loop balancing controller
std::atomic<float> com_bal_p_gain = 0;
std::atomic<float> com_bal_i_gain = 0;
std::atomic<float> com_bal_d_gain = 0;
std::atomic<float> com_vel_lpf = 0;

// flags to flip the x and y dir incase they are wrongly set
std::atomic<bool> com_x_dir = 0;
std::atomic<bool> com_y_dir = 0;

// gains for the outer loop correction on the setpoint
std::atomic<float> com_balance_pt_p_gain = 0;
std::atomic<float> com_balance_pt_i_gain = 0;
std::atomic<float> com_balance_pt_d_gain = 0;
std::atomic<float> filtered_velocity_for_outer_loop = 0;
std::atomic<float> com_outer_vel_lpf_tf = 0.05; // defaulting to 50ms

// motor control flags
std::atomic<bool> enable_flag = false;
std::atomic<bool> disable_flag = false;
std::atomic<bool> motors_enabled = false;

// update pid flag
std::atomic<bool> update_pid_flag = false;

// 0 for disable, 1 for torque foc debug, 2 for balance at given point
std::atomic<uint8_t> com_mode = 0;
uint8_t robot_mode = 0;

// balancing PID controller variables ramp and limit are set to max
// input is IMU and Output is voltage to motor
PIDController balance_pid = PIDController(0.1, 0.0, 0.0, 8.0, 8.0);

// outer slower loop to correct the setpoint
// input is the velocity of the motor and output is a small angle correction
PIDController offset_pt_pid = PIDController(0.0, 0.20, 0.0, 0.0, 0.0);

// make a separate thread for the OTA
TaskHandle_t loop_foc_task;
// make a separate thread for the MQTT publishing
TaskHandle_t mqtt_publish_task;
void mqtt_publish_thread(void *pvParameters)
{
  int interval_ms = 1000 / mqtt_update_freq_hz.load();
  while (1)
  {
    static unsigned long lastMsg = millis();
    // Handle MQTT connection
    if (!isMQTTConnected())
    {                  // Use our MQTT connection check function
      reconnectMQTT(); // Use our MQTT reconnect function
    }
    mqttLoop(); // Handle MQTT client loop (IMPORTANT)

    // Publish data periodically
    if (millis() - lastMsg > interval_ms)
    {
      // Create JSON document to send data in
      StaticJsonDocument<512> doc;

      //print enabled state
      doc["enabled"] = motors_enabled.load();
      // print the control mode
      doc["mode"] = robot_mode;
      // print target of foc
      doc["target"] = motor.target;
      // print the encoder position
      doc["pos"] = motor.shaft_angle;
      // print the encoder velocity
      doc["vel"] = motor.shaft_velocity;

      // print the IMU data
      Imu::gravity_vector_t gravity = imu.get_gravity_vector();
      doc["gravity_x"] = gravity.x;
      doc["gravity_y"] = gravity.y;

      // print balance point in radians
      float balance_point_rad = com_balance_pt_rad.load();
      float offset_rad = last_offset_rad.load();
      doc["bp_offset_rad"] = offset_rad;
      doc["balance_point_rad"] = balance_point_rad - offset_rad;
      // print the calculated error in radians
      float error_est = atan2(gravity.y, gravity.x) - balance_point_rad;
      doc["bp_error_est"] = error_est;

      // print data from the balancing PID
      float targ_v = last_balance_target_volts.load();
      doc["balance_target_volts"] = targ_v;
      
      // outer loop velocity lpf
      doc["outer_vel"] = filtered_velocity_for_outer_loop.load();

      // Serialize JSON to string
      char buffer[512];
      serializeJson(doc, buffer, sizeof(buffer));

      // Publish the message
      publishMQTT(buffer); // Use our MQTT publish function
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
    motor.move(com_motor_torque.load());
    motor.loopFOC();

    imu.loop();
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
  setupWiFi(); // Call our WiFi setup function

  // Init and calibrate the IMU
  imu.init(true);

  ArduinoOTA.setHostname("wobbler");
  ArduinoOTA.onStart(
      []()
      {
        // Stop motors on OTA
        disable_flag.store(true);
        while(motors_enabled.load())
        {
          vTaskDelay(5 / portTICK_PERIOD_MS);
        }

        // Wait for OTA to start
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        {
          type = "sketch";
        }
        else
        { // U_SPIFFS
          type = "filesystem";
        }
        Serial.println("Start updating " + type);
      });

  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd OTA Update"); });

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
  pinMode(LED_BUILTIN, OUTPUT); // BLUE LED 44
  pinMode(43, OUTPUT);          // GREEN LED 43
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
  motor.controller = MotionControlType::torque;

  // make sure no other global limits are bothering the system
  motor.velocity_limit = 50000;
  motor.current_limit = 100;
  motor.voltage_limit = 8;

  motor.LPF_velocity.Tf = 0.005; // 5ms
  motor.init();

  // LPF for outer loop
  filtered_velocity_for_outer_loop.store(0);
  com_outer_vel_lpf_tf.store(0.05); // 50ms initial tf

  // align sensor and start FOC
  sensor.voltage_calibration = 0.5;

  // calibrate the sensor and save the alignment
  sensor.calibrate(motor);
  motor.linkSensor(&sensor);

  motor.initFOC();

  setupMQTT();                                 // Call our MQTT setup function
  xTaskCreatePinnedToCore(mqtt_publish_thread, /* Task function. */
                          "MQTT_Publish",      /* String with name of task. */
                          10000,               /* Stack size in bytes. */
                          NULL,                /* Parameter passed as input of the task */
                          1,                   /* Priority of the task. */
                          &mqtt_publish_task,  /* Task handle. */
                          1);                  /* Core 1 because wifi runs on core 0 */

  // task for arduinoOTA
  xTaskCreatePinnedToCore(loop_foc_thread, "loop_foc", 10000, NULL, 1,
                          &loop_foc_task, 1);

  Serial.println("Setup complete.");
}

void loop()
{
  // TODO: add a maximum allowable balance error to shut off
  // if past controllable

  // check if mode has changed and update accordingly
  uint8_t mode = com_mode.load();
  if (mode != robot_mode)
  {
    robot_mode = mode;
    // jump table for mode differences
    switch (robot_mode)
    {
    case 1:
      motor.controller = MotionControlType::torque;
    case 0:
      disable_flag.store(true);
      break;
    case 2:
    case 3:
    {
      enable_flag.store(true);
      motor.controller = MotionControlType::torque;
      // atan2 of gravity vector is the balance point we want to aim at
      float x = imu.get_gravity_vector().x;
      float y = imu.get_gravity_vector().y;
      if (com_x_dir.load())
      {
        x = -x;
      }
      if (com_y_dir.load())
      {
        y = -y;
      }
      float balance_point_rad = atan2(y, x);
      com_balance_pt_rad.store(balance_point_rad);
    }
      break;
    default:
      break;
    }
  }

  if (update_pid_flag.load())
  {
    balance_pid.P = com_bal_p_gain.load();
    balance_pid.I = com_bal_i_gain.load();
    balance_pid.D = com_bal_d_gain.load();
    motor.LPF_velocity.Tf = com_vel_lpf.load();

    offset_pt_pid.P = com_balance_pt_p_gain.load();
    offset_pt_pid.I = com_balance_pt_i_gain.load();
    offset_pt_pid.D = com_balance_pt_d_gain.load();
    com_outer_vel_lpf_tf.store(com_outer_vel_lpf_tf.load());
    update_pid_flag.store(false);
  }
  // now handle each mode for real
  // TODO: refactor this
  switch (robot_mode)
  {
  case 1: // go on to case 2 balancing calc needed, but don't run motors
  case 2: // go on to case 3 balancing calc needed, run motor, but dont use offset
  case 3: // perform balancing calc and use motors and offset
  {
    // balance at a given point (our initialized best guess or set by a command)
    float balance_point_rad = com_balance_pt_rad.load();
    float offset_rad = 0;
    // in debug mode, the offset CAN be set by user, but in mode 3 we will overwrite

    // calculate the offset based on the outer loop
    // the offset should aim to reduce the velocity to zero
    float raw_velocity = motor.shaft_velocity;
    float tf = com_outer_vel_lpf_tf.load();
    float dt = 0.001; // 5ms  // TODO: make this adjust based on the loop time
    float alpha = 1.0f; // default for tf is invalid
    if (tf > 0)
    {
      alpha = constrain( (dt / (tf + dt)), 0.0, 1.0 ) ;
    }
    float last_vel = filtered_velocity_for_outer_loop.load();
    filtered_velocity_for_outer_loop.store((1 - alpha) * raw_velocity + alpha * last_vel);
    offset_rad = offset_pt_pid(filtered_velocity_for_outer_loop);
    if (robot_mode == 3)
    {
      // robot mode 3 === FULL ENABLED CONTROL LOOPS - Balance point and offset correction
      // apply offset
      balance_point_rad += offset_rad;
    }
    else
    {
      // robot mode 2 === ENABLED CONTROL LOOPS - Balance point only
      // only apply offset if the user has explicitly set it for debugging - not being calc.
      offset_rad = com_balance_offset_rad.load();
    }

    // find error based on gravity vector x and y components and current setpoint
    // the linear approximation is good enough.  assume small angle approximation
    float x = imu.get_gravity_vector().x;
    float y = imu.get_gravity_vector().y;

    float calculated_error_rad = atan2(y, x) - balance_point_rad;

    /*  ------------------------------------------------------------------
     *  get output of balancing pid (volts)
     *  note that the frequency of calling balance_pid DEFINES the freq of
     *  the control loop */
    float balance_target_volts = balance_pid(calculated_error_rad);

    // set motor torque based on both loops
    // if abs value is within .005 volts, zero out the signal
    if (abs(balance_target_volts) < 0.015)
    {
      balance_target_volts = 0;
    }
    else
    {
      float ff = com_feedforward.load();
      balance_target_volts += (balance_target_volts > 0) ? ff : -(ff);
    }

    // set the motor torque
    com_motor_torque.store(balance_target_volts);

    // store the last values for debugging
    last_balance_target_volts.store(balance_target_volts);
    last_offset_rad.store(offset_rad);
  }
    break;
  case 0:
    // do nothing
    break;
  default:
    break;
  }

  //   Handle OTA updates
  ArduinoOTA.handle();
 
  // delay task for 1 ms
  vTaskDelay(1 / portTICK_PERIOD_MS);
}