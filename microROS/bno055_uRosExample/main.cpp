#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/vector3.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

//------------------------------------
//           ROS variables
//------------------------------------
rcl_publisher_t counter_pub;
rcl_publisher_t imu_pub;

rcl_subscription_t led_sub;

std_msgs__msg__Int32 counter_msg;
geometry_msgs__msg__Vector3 accel_msg;
std_msgs__msg__Bool led_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t counter_timer;
rcl_timer_t imu_timer;

//------------------------------------
//             IMU setup
//------------------------------------
#define SDA_PIN 5
#define SCL_PIN 4
#define IMU_RATE_MS 100       // 10 Hz

Adafruit_BNO055 myIMU;

//------------------------------------
//        Helper macros / funcs
//------------------------------------
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; }

void error_loop() {
  while (true) { delay(100); }
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
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accel_msg.x = acc.x();
  accel_msg.y = acc.y();
  accel_msg.z = acc.z();
  RCSOFTCHECK(rcl_publish(&imu_pub, &accel_msg, NULL));
}

// -- suscriptor del LED
void led_cb(const void * msg_in) {
  const std_msgs__msg__Bool * m = static_cast<const std_msgs__msg__Bool *>(msg_in);
  digitalWrite(1, m->data ? HIGH : LOW);
}

//------------------------------------
//               setup
//------------------------------------
void setup() {
  // Serial ↔ micro-ROS
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // LED
  pinMode(1, OUTPUT);

  // IMU I²C
  if (!Wire.begin(SDA_PIN, SCL_PIN)) {
    Serial.println("I²C init failed"); error_loop();
  }
  if (!myIMU.begin()) {
    Serial.println("BNO055 not detected"); error_loop();
  }
  myIMU.setExtCrystalUse(true);

  //--------------------------------
  //   micro-ROS entities
  //--------------------------------
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_s3_node", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(
      &counter_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "counter"));
  RCCHECK(rclc_publisher_init_default(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "imu/accel"));

  // Subscriber
  RCCHECK(rclc_subscription_init_default(
      &led_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control"));

  // Timers
  RCCHECK(rclc_timer_init_default(
      &counter_timer, &support,
      RCL_MS_TO_NS(1000), counter_timer_cb));
  RCCHECK(rclc_timer_init_default(
      &imu_timer, &support,
      RCL_MS_TO_NS(IMU_RATE_MS), imu_timer_cb));

  // Executor (4 handles: 2 timers + 1 sub)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &counter_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &led_sub, &led_msg, &led_cb, ON_NEW_DATA));

  // Inicializa mensajes
  counter_msg.data = 0;
}

//------------------------------------
//               loop
//------------------------------------
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);   // evita watchdog
}
