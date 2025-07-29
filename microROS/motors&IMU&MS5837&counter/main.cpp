#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "MS5837.h"

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/vector3.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

//------------------------------------
//           Motor Pins
//------------------------------------
int enMotorPins[6] = {15, 3, 2, 37, 14, 9};
int inMotorPins[6] = {16, 18, 42, 40, 12, 11};
int inMotorPins2[6] = {17, 8, 41, 39, 13, 10}; 


//------------------------------------
//           ROS variables
//------------------------------------
// micro-ROS core objects
rcl_publisher_t counter_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t ms5837_pub;
rcl_subscription_t subscriber;

// Mensaje del suscriptor y counter_pub
std_msgs__msg__Int32MultiArray motor_msg;
std_msgs__msg__Int32 counter_msg;
geometry_msgs__msg__Vector3 imu_msg;
geometry_msgs__msg__Vector3 ms5837_pub_msg;

// micro-ROS support and executor
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t counter_timer;
rcl_timer_t imu_timer;
rcl_timer_t ms5837_timer;

//------------------------------------
//             IMU setup
//------------------------------------
#define SDA_PIN_IMU 7
#define SCL_PIN_IMU 6
#define IMU_RATE_MS 200       // 5 Hz

Adafruit_BNO055 myIMU;

//------------------------------------
//             MS5837 setup
//------------------------------------
#define SDA_PIN_MS5837 4
#define SCL_PIN_MS5837 5
#define MS5837_RATE_MS 500       // 2 Hz
MS5837 sensor;

//------------------------------------
//        Helper macros / funcs
//------------------------------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

//------------------------------------
//          ROS callbacks
//------------------------------------
// -- publ. contador cada 1 s
void counter_timer_cb(rcl_timer_t *, int64_t) {
  RCSOFTCHECK(rcl_publish(&counter_pub, &counter_msg, NULL));
  counter_msg.data++;
}

// -- publ. IMU cada 100 ms
void imu_timer_cb(rcl_timer_t *, int64_t) {
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu_msg.x = euler.x();
  imu_msg.y = euler.y();
  imu_msg.z = euler.z();
  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

// -- publ. MS5837 cada 500 ms
void ms5837_timer_cb(rcl_timer_t *, int64_t) {
  sensor.read();

  // Save pressure, temperature, depth, and altitude
  ms5837_pub_msg.x = sensor.pressure();
  ms5837_pub_msg.y = sensor.temperature();
  ms5837_pub_msg.z = sensor.depth();
  RCSOFTCHECK(rcl_publish(&ms5837_pub, &ms5837_pub_msg, NULL));

  // int32_t pressure    = (int32_t)(sensor.pressure() * 100);    // mbar * 100
  // int32_t temperature = (int32_t)(sensor.temperature() * 100); // °C * 100
  // int32_t depth       = (int32_t)(sensor.depth() * 100);       // m * 100
  // int32_t altitude    = (int32_t)(sensor.altitude() * 100);    // m * 100
  
  // // Prepare message
  // if (ms5837_pub_msg.data.data == NULL) {
  //   ms5837_pub_msg.data.data = (int32_t *)malloc(4 * sizeof(int32_t));
  //   ms5837_pub_msg.data.capacity = 4;
  // }
  
  // // Fill the message with data
  // ms5837_pub_msg.data.data[0] = pressure;
  // ms5837_pub_msg.data.data[1] = temperature;
  // ms5837_pub_msg.data.data[2] = depth;
  // ms5837_pub_msg.data.data[3] = altitude;
  // ms5837_pub_msg.data.size = 4;

  // RCSOFTCHECK(rcl_publish(&ms5837_pub, &ms5837_pub_msg, NULL));
}

// Lógica de control para el motor
void control_motor(int index, int direction, int speed) {
  if (index < 0 || index >= 6) return;
  speed = constrain(speed, 0, 255);

  if (direction == 1) {
    digitalWrite(inMotorPins[index], HIGH);
    digitalWrite(inMotorPins2[index], LOW);
  } else if (direction == -1) {
    digitalWrite(inMotorPins[index], LOW);
    digitalWrite(inMotorPins2[index], HIGH);
  } else {
    digitalWrite(inMotorPins[index], LOW);
    digitalWrite(inMotorPins2[index], LOW);
  }

  analogWrite(enMotorPins[index], speed);
}

// Callback del subscriber
void motor_command_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *) msgin;
  if (msg->data.size < 3) return;

  int index     = msg->data.data[0];
  int direction = msg->data.data[1];
  int speed     = msg->data.data[2];

  control_motor(index, direction, speed);
}

void setup() {
  // Serial para micro-ROS
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // IMU I²C
  if (!Wire.begin(SDA_PIN_IMU, SCL_PIN_IMU)) {
    Serial.println("IMU I²C init failed"); error_loop();
  }
  if (!myIMU.begin()) {
    Serial.println("BNO055 not detected"); error_loop();
  }
  myIMU.setExtCrystalUse(true);
  delay(2000);

  // MS5837 I²C
  if (!Wire.begin(SDA_PIN_MS5837, SCL_PIN_MS5837)) {
    Serial.println("MS5837 I²C init failed"); error_loop();
  }
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  delay(2000);
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  // Configurar pines
  for (int i = 0; i < 6; i++) {
    pinMode(enMotorPins[i], OUTPUT);
    pinMode(inMotorPins[i], OUTPUT);
    pinMode(inMotorPins2[i], OUTPUT);
    digitalWrite(inMotorPins[i], LOW);
    digitalWrite(inMotorPins2[i], LOW);
  }

  
  //--------------------------------
  //   Message Initialization
  //--------------------------------
  counter_msg.data = 0;
  imu_msg.x = 0.0;
  imu_msg.y = 0.0;
  imu_msg.z = 0.0;
  ms5837_pub_msg.x = 0.0;
  ms5837_pub_msg.y = 0.0;
  ms5837_pub_msg.z = 0.0;
  motor_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t));
  motor_msg.data.size = 3;
  motor_msg.data.capacity = 3;

  //--------------------------------
  //   micro-ROS entities
  //--------------------------------
  allocator = rcl_get_default_allocator();  
  
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "motor_command"));

  // Publishers
  RCCHECK(rclc_publisher_init_default(
      &counter_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "counter_pub"));
  RCCHECK(rclc_publisher_init_default(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "imu/euler_angles"));
  RCCHECK(rclc_publisher_init_default(
      &ms5837_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "ms5837/data"));
  
  // Timers
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
      &counter_timer, &support,
      RCL_MS_TO_NS(timer_timeout), counter_timer_cb));
  RCCHECK(rclc_timer_init_default(
      &imu_timer, &support,
      RCL_MS_TO_NS(IMU_RATE_MS), imu_timer_cb));
  RCCHECK(rclc_timer_init_default(
      &ms5837_timer, &support,
      RCL_MS_TO_NS(MS5837_RATE_MS), ms5837_timer_cb));

  // Inicializar executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &counter_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ms5837_timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &motor_msg,
    &motor_command_callback,
    ON_NEW_DATA));
}

//------------------------------------
//               loop
//------------------------------------
void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

