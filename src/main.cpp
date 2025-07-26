#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Pines del hardware
int enMotorPins[6]   = {15, 3, 2, 37, 14, 9};
int inMotorPins[6]   = {16, 18, 42, 40, 12, 11};
int inMotorPins2[6]  = {17, 8, 41, 39, 13, 10};

// micro-ROS core objects
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Mensaje del suscriptor
std_msgs__msg__Int32MultiArray motor_msg;

// Macros de seguridad
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

void error_loop() {
  while (1) {
    delay(100);
  }
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

  // Configurar pines
  for (int i = 0; i < 6; i++) {
    pinMode(enMotorPins[i], OUTPUT);
    pinMode(inMotorPins[i], OUTPUT);
    pinMode(inMotorPins2[i], OUTPUT);
    digitalWrite(inMotorPins[i], LOW);
    digitalWrite(inMotorPins2[i], LOW);
  }

  allocator = rcl_get_default_allocator();

  // Inicializar soporte y nodo
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_control_node", "", &support));

  // Inicializar suscriptor
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "motor_command"));

  // Inicializar executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &motor_msg,
    &motor_command_callback,
    ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
