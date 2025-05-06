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
#define VERSION "1.0.445" // updated dynamically from python script

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

// motor parameters
int pole_pairs = 11;
float phase_resistance = 2.15;

// Setup the motor and driver
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance);
BLDCDriver6PWM driver =
    BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);

// make encoder for simplefoc
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI encoder0(CH0_ENC_CS);

// calibrated sensor object from simplefoc
CalibratedSensor sensor = CalibratedSensor(encoder0);

// IMU
Imu::Imu imu(0.01);
float gyro_z_rads = 0;

// atomic variable to adjust mqtt update frequency
std::atomic<uint8_t> mqtt_update_freq_hz = 10; // default to 100ms (10Hz)
std::atomic<u_long> time_checker_imu = 0; // for checking the time in the imu.loop()
std::atomic<u_long> time_checker_foc = 0; // for checking the time in the foc loop
unsigned long last_time_foc = 0; // for checking the time in the loops
unsigned long last_time_imu = 0; // for checking the time in the imu loop
std::atomic<float> imu_filter = 0.01; //madgwick filter gain on imu

std::atomic<float> com_vel_p = 0.3;
std::atomic<float> com_vel_i = 0.0;
std::atomic<float> com_vel_d = 0.0000;
std::atomic<float> com_vel_lpf = 0.025;
std::atomic<float> last_commanded_vel_rads = 0.0;
std::atomic<float> com_target_debug = 0.0; // target for debugging

// global atomic variable for the motor stuff to be set by mqtt
// target will not be set by us except for debugging
std::atomic<float> com_balance_pt_rad = 0.0;     // balance point in radians
std::atomic<float> com_balance_offset_rad = 0.0; // offset in radians
std::atomic<float> last_balance_target_delta_rads = 0.0;
std::atomic<float> last_offset_rad = 0.0;

// gains for the inner loop balancing controller
std::atomic<float> com_bal_p_gain = 0.0;
std::atomic<float> com_bal_i_gain = 0.0;
std::atomic<float> com_bal_d_gain = 0.0;

// flags to flip the x and y dir incase they are wrongly set
std::atomic<bool> com_x_dir = 0;
std::atomic<bool> com_y_dir = 0;

// gains for the outer loop correction on the setpoint
std::atomic<float> com_balance_pt_p_gain = 0.00;
std::atomic<float> com_balance_pt_i_gain = 0.001;
std::atomic<float> com_balance_pt_d_gain = 0.000;
std::atomic<float> filtered_velocity_for_outer_loop = 0.0;
std::atomic<float> com_outer_vel_lpf_tf = 0.5; // defaulting to 500ms

std::atomic<float> com_feedfwd_scale = 0.6;
std::atomic<float> com_fw_rotation_scale = 0.3;
std::atomic<float> com_fw_gravity_scale = 0.1;

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
const float delta_v_ramp_lim = 200.0;
const float max_pid_output = 1000;
PIDController balance_pid = PIDController(0.0, 0.0, 0.5, delta_v_ramp_lim, max_pid_output);

// outer slower loop to correct the setpoint
// input is the velocity of the motor and output is a small angle correction
LowPassFilter offset_lpf = LowPassFilter(0.5);
PIDController offset_pt_pid = PIDController(0.000, 0.00, 0.0000, 0, 100000);
LowPassFilter gyro_lpf = LowPassFilter(0.05);
LowPassFilter effort_lpf = LowPassFilter(0.5);
std::atomic<float> accumulated_effort = 0;

// make a separate thread for the OTA
TaskHandle_t loop_foc_task;
float foc_ff = 0;
float foc_vel = 0;
unsigned long foc_now_time = 0;
unsigned long foc_elapsed_time = 0;

// control loop variables
float balance_point_rad = 0;
float balance_point_x = 0;
float balance_point_y = 0;
float max_balance_point = 0;
float min_balance_point = 0;
float offset_rad_rate = 0;
float raw_velocity = 0;
float tf = 0;
float dt = 0;
float alpha = 0;

float last_vel = 0;
float x = 0;
float y = 0;
std::atomic<float> gyro_z = 0;
float calculated_error_rad = 0;
float balance_target_delta_rads = 0;
float new_target_vel = 0;

unsigned long now_time = 0;
unsigned long elapsed_time = 0;

// for calculating feed forward energy
std::atomic<float> w_feedback = 0;
std::atomic<float> w_feedfwd = 0;
const float _M = 0.263; // measured in kg on my kitchen scale
const float _H = 0.1; // measured from corner to center of mass in m
const float _I_WHEEL = 0.000190 + 0.000020 + 0.00010; // inertia of ring, rotor, and holder in kg*m^2
const float _I_TOTAL_ROBOT = _M * _H * _H; // inertia of whole robot about axis of tipping
const float _G = 9.81;
const float _E_GMAX = _M*_G*_H; // this is the max potential energy
float k_energy_now = 0;
float gp_energy_now = 0;
float delta_k_energy = 0;
std::atomic<float> k_coulomb_damp = 0; // for calculating torque on motor due to constant damp
std::atomic<float> k_viscous_damp = 0; // for calculating torque on motor due to proportional damp
float damping_energy_lost_now = 0;
std::atomic<float> theta = 0; // angle in rads measured from target balance point
float offset_total = 0;
float last_offset_total = 0;
uint16_t timer_offset_rads = 0;


// esp_timer_handle_t foc_timer;
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

      // print enabled state
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
      doc["gravity_x"] = x;
      doc["gravity_y"] = y;
      doc["gyro_z"] = gyro_z.load();

      // print balance point in radians
      doc["bp_offset_rad"] = last_offset_rad.load();
      doc["balance_point_rad"] = balance_point_rad;
      // print the calculated error in radians
      float error_est = -theta.load();
      doc["bp_error_est"] = error_est;

      // print data from the balancing PID
      float targ_v = last_balance_target_delta_rads.load();
      doc["balance_target_volts"] = targ_v;

      // outer loop velocity lpf
      doc["effort_total"] = accumulated_effort.load();
      
      // print time of control loop
      doc["imu_time"] = time_checker_imu.load();
      doc["feed_fwd"] = w_feedfwd.load();
      doc["theta"] = theta.load();

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
    // if the last_commanded_vel abs is less than the abs of the feedforward,
    // set the target to the feedforward
    foc_ff = 0; // for gimbal motors this isn't needed
    foc_vel = last_commanded_vel_rads.load();
    if (abs(foc_vel) < abs(foc_ff) && foc_vel != 0.0)
    {
      foc_vel = (foc_vel / abs(foc_vel)) * foc_ff;
    }
    motor.move(foc_vel); // set the target velocity to the motor
    motor.loopFOC();

  }
}

float dot_product(float &a1, float &b1, float &a2, float &b2){
  // Normalize this stuff
  float denom = sqrt(a1*a1 + b1*b1) * sqrt(a2*a2 + b2*b2);
  float dot;
  if (denom == 0){
    dot = 1.0;
  }
  else
    dot = (a1 * a2 + b1 * b2) / denom;
    constrain(dot, -1.0, 1.0);
  return dot;
}

float cross(float &a1, float &b1, float &a2, float &b2){
  return(a1*b2 - a2*b1);
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
        while (motors_enabled.load())
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
  driver.voltage_power_supply = 15;
  driver.voltage_limit = 15;
  driver.init();

  // link motor to driver and set up
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;

  // make sure no other global limits are bothering the system
  motor.velocity_limit = 1000;
  motor.current_limit = 15.0;
  motor.voltage_limit = 15.0;

  motor.PID_velocity.P = 1.2;
  motor.PID_velocity.I = 0.0;
  motor.PID_velocity.D = 0.00000;
  motor.PID_velocity.limit = 100;

  motor.LPF_velocity.Tf = 0.015;
  motor.init();

  // LPF for outer loop
  filtered_velocity_for_outer_loop.store(0);
  accumulated_effort.store(0);
  com_outer_vel_lpf_tf.store(1.0); // 50ms initial tf

  // align sensor and start FOC
  sensor.voltage_calibration = 7.0;

  // calibrate the sensor and save the alignment
  sensor.calibrate(motor);
  motor.linkSensor(&sensor);

  motor.initFOC();

  setupMQTT();                                 // Call our MQTT setup function
  xTaskCreatePinnedToCore(mqtt_publish_thread, /* Task function. */
                          "MQTT_Publish",      /* String with name of task. */
                          10000,               /* Stack size in bytes. */
                          NULL,                /* Parameter passed as input of the task */
                          5,                   /* Priority of the task. */
                          &mqtt_publish_task,  /* Task handle. */
                          0);                  /* Core 1 because wifi runs on core 0 */

  // task for motor controls
  xTaskCreatePinnedToCore(loop_foc_thread, "loop_foc", 10000, NULL, 1,
                          &loop_foc_task, 1);

  // start the motor disabled
  motor.disable();
  motors_enabled.store(false);
  Serial.println("Setup complete.");
  last_time_imu = micros();
  last_time_foc = micros();
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
      balance_point_x = x;
      balance_point_y = y;
    case 0:
      motor.controller = MotionControlType::velocity;
      // set targets to 0
      last_commanded_vel_rads.store(0.0);
      com_balance_offset_rad.store(0.0);
      accumulated_effort.store(0);
      // clear pid
      motor.PID_velocity.reset();
      balance_pid.reset();
      offset_pt_pid.reset();
      disable_flag.store(true);
      break;
    case 2:
    case 3:
    {
      // set targets to 0
      last_commanded_vel_rads.store(0.0);
      com_balance_pt_rad.store(0.0);
      com_balance_offset_rad.store(0.0);
      accumulated_effort.store(0);
      offset_total = 0;
      last_offset_total = 0;
      offset_rad_rate = 0;
      // clear pid
      motor.PID_velocity.reset();
      balance_pid.reset();
      offset_pt_pid.reset();
      enable_flag.store(true);
      motor.controller = MotionControlType::velocity;
      // atan2 of gravity vector is the balance point we want to aim at
      x = imu.get_gravity_vector().x;
      y = imu.get_gravity_vector().y;
      if (com_x_dir.load())
      {
        x = -x;
      }
      if (com_y_dir.load())
      {
        y = -y;
      }
      balance_point_x = x;
      balance_point_y = y;
    }
    break;
    default:
      break;
    }
  }

  if (update_pid_flag.load())
  {
    // motor velocity pid gains
    motor.PID_velocity.P = com_vel_p.load();
    motor.PID_velocity.I = com_vel_i.load();
    motor.PID_velocity.D = com_vel_d.load();
    motor.LPF_velocity.Tf = com_vel_lpf.load();

    // balancing pid gains
    balance_pid.P = com_bal_p_gain.load();
    balance_pid.I = com_bal_i_gain.load();
    balance_pid.D = com_bal_d_gain.load();

    offset_pt_pid.P = com_balance_pt_p_gain.load();
    offset_pt_pid.I = com_balance_pt_i_gain.load();
    offset_pt_pid.D = com_balance_pt_d_gain.load();

    // clear the pids
    motor.PID_velocity.reset();
    balance_pid.reset();
    offset_pt_pid.reset();
    update_pid_flag.store(false);
    effort_lpf.Tf = 0.0000001;
    effort_lpf(0.0);
    accumulated_effort.store(0);
  }
  // now handle each mode for real
  // balance at a given point (our best guess pre-offsets)

  // in debug mode, the offset CAN be set by user, but in mode 3 we will overwrite
  // calculate the offset based on the outer loop
  // the offset should aim to reduce the velocity to zero
  raw_velocity = motor.shaft_velocity;
  filtered_velocity_for_outer_loop.store(offset_lpf(raw_velocity)); // store for mqtt panel
  
  effort_lpf.Tf = com_outer_vel_lpf_tf.load();
  
  // normally delay the task, but since imu takes so dang long, just assume 2ms ish delay
  imu.setGain(imu_filter.load());
  imu.loop();

  // TODO: THE OFFSET NEEDS TO BE CALCULATED INTO THE BALANCE POINT FOR CONTROL MODE 3 TO WORKKKKKK
  // find error based on gravity vector x and y components and current setpoint
  x = imu.get_gravity_vector().x;
  y = imu.get_gravity_vector().y;
  float dot = dot_product(x, y, balance_point_x, balance_point_y);
  float acos_val = acos(dot);

  // this constraint saves us from acos() returning nans
  if (dot >= 1.0){
    acos_val = 0.0;
  }
  else if (dot <= -1)
  {
    acos_val = _PI;
  }

  float theta_signed = copysign(acos_val, cross(x,y,balance_point_x, balance_point_y));

  offset_rad_rate = effort_lpf(offset_pt_pid(last_vel) / 100000.0);
  // offset_total = constrain(offset_total + offset_rad_rate, -1.0, 1.0);

  // in mode 2, overwrite the offset with the commanded offset
  if (mode == 2)
  {
    offset_rad_rate = com_balance_offset_rad.load();
    accumulated_effort.store(0);
  }

  offset_total += offset_rad_rate;

  theta.store(constrain(theta_signed - offset_total, -1.0, 1.0)); 

  calculated_error_rad = -theta.load(); // error is negative theta

  // first find the feed forward deltaV based on the Energy Equations.
  // Assuming:
  // grav pot energy = m*g*cos(theta)  --- where m is mass estimage of robot, g is 10, and theta is error.
  // kin energy = .5* I*theta_dot^2  --- where I is robot inertia estimate and theta_dot is current robot tipping vel. from gyro
  
  // find kinetic energy from gyro data - the robots rotation is in the X-Y plane so we take gyro Z data
  
  gyro_z.store(gyro_lpf(imu.get_raw_gyro_z()*SENSORS_DPS_TO_RADS));
  gyro_z_rads = constrain(gyro_z.load(), -10.0, 10.0);
  
  // this energy term is how much energy the robot would have in its body rotation, if its body were moving towards the offset at the given rate
  // delta w needed to add ang.vel to robot body towards theta
  float ke_robot_target = .5 * _I_TOTAL_ROBOT * offset_rad_rate * offset_rad_rate;
  float w_feedfwd_target_theta_dot = sqrt(2.0 * abs(ke_robot_target)/_I_WHEEL);  //TODO:  the math can be simplified on all of these terms.
  
  k_energy_now = .5 * _I_TOTAL_ROBOT * gyro_z_rads * gyro_z_rads; // this is energy in the rotation of the robot body right now
  float ke_rotation = constrain((k_energy_now), 0, _E_GMAX/2); // going to limit this to half the GPE because that seems like a good ceil.
  float w_feedfwd_rotation = sqrt(2.0 * abs(ke_rotation) / _I_WHEEL);  // delta w needed to cancel robot body momentum
  
  gp_energy_now = _M * _G * cos(theta.load()) * _H;  // this is how much gravitational potential energy we have between the max height and the current position
  float delta_k_energy_grav = (_E_GMAX - gp_energy_now); // this is the component of energy we need to add for fixing the height of the robot
  float w_feedfwd_grav = sqrt(2.0 * abs(delta_k_energy_grav)/ _I_WHEEL); // w needed to match energy difference in gpe

  // get signs for all the components

  // should always be moving robot body towards theta, implies wheel should always be opposite of theta:
  w_feedfwd_target_theta_dot = -copysign(w_feedfwd_target_theta_dot, offset_rad_rate); 
  // always counteracting the rotational diff, which we get from gyro data:
  w_feedfwd_rotation = -copysign(w_feedfwd_rotation, gyro_z_rads); 
  // always moving robot body to counteract the GPE diff, implies wheel should always be opposite of theta:
  w_feedfwd_grav = -copysign(w_feedfwd_grav, theta_signed);

  // scale feedfwd to gain
  // TODO: stop using coulomb damping parameter as the scaling for theta dot term - I'm just lazy and its available in the gui to tune with.
  w_feedfwd.store(w_feedfwd_grav * com_fw_gravity_scale.load() + w_feedfwd_rotation * com_fw_rotation_scale.load() + w_feedfwd_target_theta_dot*k_coulomb_damp.load());

  /*  ------------------------------------------------------------------
   *  get output of balancing pid (delta velocity == delta rad/s)
   *  note that the frequency of calling balance_pid DEFINES the freq of
   *  the control loop */
  w_feedback.store(balance_pid(calculated_error_rad));
  balance_target_delta_rads = w_feedback.load() + w_feedfwd.load();
  
  // finally, if the output is too large, set it to the max
  balance_target_delta_rads = constrain(balance_target_delta_rads, -200.0, 200.0);

  // this output is what we will integrate and call our accumulated effort

  if (robot_mode == 3){
    accumulated_effort.store(effort_lpf(last_vel));
  }

  // if (abs(balance_target_delta_rads) < .005){
  //   balance_target_delta_rads = 0.0;
  // }

  // add the delta V to the current velocity to get the new velocity
  new_target_vel = (last_commanded_vel_rads.load() + balance_target_delta_rads);

  // set the motor commands with our calculated new target unless we are in mode 1
  // in which just pass the debug target to the velocity controller
  if (robot_mode == 1){
    last_commanded_vel_rads.store(com_target_debug.load());
  }
  else if (robot_mode == 0){
    //do nothing
  }
  else if (robot_mode == 2 || robot_mode == 3){
    last_commanded_vel_rads.store(new_target_vel);
  }

  // store the last values for debugging and logging
  last_balance_target_delta_rads.store(balance_target_delta_rads);
  last_offset_rad.store(offset_rad_rate);

  // Handle OTA updates
  ArduinoOTA.handle();

  last_vel = motor.shaft_velocity;

  now_time = micros();
  elapsed_time = now_time - last_time_imu;
  last_time_imu = now_time;
  time_checker_imu.store(elapsed_time);
}