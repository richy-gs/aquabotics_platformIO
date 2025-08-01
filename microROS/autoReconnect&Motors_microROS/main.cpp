// Include Libraries to be used
#include <Arduino.h>
#include <micro_ros_platformio.h>     // micro-ros-platformio library

#include <rcl/rcl.h>                  // Core ROS 2 Client Library (RCL) for node management
#include <rcl/error_handling.h>       // Error handling utilities for Micro-ROS
#include <rclc/rclc.h>                // Micro-ROS Client library for embedded devices
#include <rclc/executor.h>            // Micro-ROS Executor to manage callbacks

#include <rmw_microros/rmw_microros.h>// ROS Middleware for Micro-ROS
#include <stdio.h>                    // Standard I/O library

#include <std_msgs/msg/int32_multi_array.h> // Predefined ROS 2 message type
#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Pines del hardware
// int enMotorPins[6] = {15, 3, 2, 37, 14, 9};
// int inMotorPins[6] = {16, 18, 42, 40, 12, 11};
// int inMotorPins2[6] = {17, 8, 41, 39, 13, 10}; 

int enMotorPins[6]  = {15, 37,  3,  9,  2, 14};
int inMotorPins[6]  = {16, 40,  8, 11, 41, 12};
int inMotorPins2[6] = {17, 39, 18, 10, 42, 13};

// micro-ROS core objects
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_subscription_t control_subscriber;

// Mensaje del suscriptor y publisher
std_msgs__msg__Int32MultiArray motor_msg;
std_msgs__msg__Int32MultiArray control_msg;
std_msgs__msg__Int32 msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Macros de seguridad
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Macro to use
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
  } while (0)

// Defines State Machine States
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Create entity functions
bool create_entities();
void destroy_entities();

// Define callback functions
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
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

void control_callback(const void *msgin) {
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

  // allocator = rcl_get_default_allocator();  
  
  //create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  // RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Inicializar suscriptor
  // RCCHECK(rclc_subscription_init_default(
  //   &subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
  //   "individual/motor_command"));
  // RCCHECK(rclc_subscription_init_default(
  //   &control_subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
  //   "xbox/motor_command"));

  // // create publisher
  // RCCHECK(rclc_publisher_init_default(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "micro_ros_platformio_node_publisher"));

  // // create timer,
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // // Inicializar executor
  // RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  // RCCHECK(rclc_executor_add_subscription(
  //   &executor,
  //   &subscriber,
  //   &motor_msg,
  //   &motor_command_callback,
  //   ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_subscription(
  //   &executor,
  //   &control_subscriber,
  //   &control_msg,
  //   &control_callback,
  //   ON_NEW_DATA));

  msg.data = 0; // Inicializar el mensaje del publisher
  motor_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t));
  motor_msg.data.size = 3;
  motor_msg.data.capacity = 3;
  control_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t));
  control_msg.data.size = 3;
  control_msg.data.capacity = 3;
}

void loop() {
  // delay(100);
  // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));

  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    }
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }
}

// Create entities
bool create_entities()
{
  // Initializes memory allocation for Micro-ROS operations
  allocator = rcl_get_default_allocator();

  // Creates a ROS 2 support structure
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_sub_node", "", &support));

  // Inicializar suscriptor
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "individual/motor_command"));
  RCCHECK(rclc_subscription_init_default(
    &control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "xbox/motor_command"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "counter_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));


  // Create zero-initialized executor
  executor = rclc_executor_get_zero_initialized_executor();

  // Initialize the executor
  // Inicializar executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &motor_msg,
    &motor_command_callback,
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &control_subscriber,
    &control_msg,
    &control_callback,
    ON_NEW_DATA));

  return true;
}

// Destroy entities
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscriber, &node);
  rcl_subscription_fini(&control_subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}