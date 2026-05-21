#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include "esp_heap_caps.h"

// ======================================================
// CONFIG
// ======================================================

#define WIFI_SSID   ""
#define WIFI_PASS   ""


#define AGENT_IP    "192.168.19.101"
#define AGENT_PORT  8888

#define DOMAIN_ID   42

#define SERVO_PIN   4
#define INPUT_PIN   12
#define LED_PIN     22

// ======================================================
// ROS OBJECTS
// ======================================================

rcl_subscription_t sub_sequence;
rcl_subscription_t sub_hold;

std_msgs__msg__Int32 msg_seq;
std_msgs__msg__Int32 msg_hold;

rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

Servo s1;

// ======================================================
// STATE VARIABLES
// ======================================================

int32_t persistent_angle = 90;

unsigned long sequence_start_time = 0;

enum ServoState {
  IDLE,
  MOVING_TO_TARGET,
  RETURNING_TO_180
};

ServoState servo_state = IDLE;

bool last_pin_state = HIGH;

enum DeviceState {
  WAITING_WIFI,
  AGENT_DISCONNECTED,
  AGENT_CONNECTED
};

DeviceState device_state = WAITING_WIFI;

// ======================================================
// SAFE CLEANUP
// ======================================================

void destroy_entities() {

  Serial.println("Destroying ROS entities...");

  rmw_context_t * rmw_context =
      rcl_context_get_rmw_context(&support.context);

  rmw_uros_set_context_entity_destroy_session_timeout(
      rmw_context,
      0);

  rclc_executor_fini(&executor);

  rcl_subscription_fini(&sub_sequence, &node);
  rcl_subscription_fini(&sub_hold, &node);

  rcl_node_fini(&node);

  rclc_support_fini(&support);

  // Prevent stale handles
  memset(&executor, 0, sizeof(executor));
  memset(&node, 0, sizeof(node));
  memset(&support, 0, sizeof(support));
}

// ======================================================
// CREATE ENTITIES SAFELY
// ======================================================

bool create_entities() {

  allocator = rcl_get_default_allocator();

  rcl_init_options_t opts =
      rcl_get_zero_initialized_init_options();

  // ---------------- INIT OPTIONS ----------------

  if (rcl_init_options_init(&opts, allocator) != RCL_RET_OK) {
    Serial.println("Failed init_options_init");
    return false;
  }

  if (rcl_init_options_set_domain_id(
          &opts,
          DOMAIN_ID) != RCL_RET_OK) {

    Serial.println("Failed set_domain_id");

    rcl_init_options_fini(&opts);
    return false;
  }

  rmw_init_options_t* rmw_opts =
      rcl_init_options_get_rmw_init_options(&opts);

  rmw_uros_options_set_client_key(
      (uint32_t)random(),
      rmw_opts);

  // ---------------- SUPPORT ----------------

  if (rclc_support_init_with_options(
          &support,
          0,
          NULL,
          &opts,
          &allocator) != RCL_RET_OK) {

    Serial.println("Failed support init");

    rcl_init_options_fini(&opts);
    return false;
  }

  // IMPORTANT
  rcl_init_options_fini(&opts);

  // ---------------- NODE ----------------

  if (rclc_node_init_default(
          &node,
          "lolin32_servo_node",
          "",
          &support) != RCL_RET_OK) {

    Serial.println("Failed node init");

    rclc_support_fini(&support);
    return false;
  }

  // ---------------- SUBSCRIPTIONS ----------------

  if (rclc_subscription_init_best_effort(
          &sub_sequence,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "trigger") != RCL_RET_OK) {

    Serial.println("Failed sub_sequence");

    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return false;
  }

  if (rclc_subscription_init_best_effort(
          &sub_hold,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "servo_hold") != RCL_RET_OK) {

    Serial.println("Failed sub_hold");

    rcl_subscription_fini(&sub_sequence, &node);
  
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return false;
  }

  // ---------------- EXECUTOR ----------------

  if (rclc_executor_init(
          &executor,
          &support.context,
          2,
          &allocator) != RCL_RET_OK) {

    Serial.println("Failed executor init");

    rcl_subscription_fini(&sub_sequence, &node);
    rcl_subscription_fini(&sub_hold, &node);

    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return false;
  }

  // ---------------- ADD CALLBACKS ----------------

  rclc_executor_add_subscription(
      &executor,
      &sub_sequence,
      &msg_seq,
      &callback_sequence,
      ON_NEW_DATA);

  rclc_executor_add_subscription(
      &executor,
      &sub_hold,
      &msg_hold,
      &callback_hold,
      ON_NEW_DATA);

  Serial.println("ROS entities created");

  return true;
}

// ======================================================
// SERVO LOGIC
// ======================================================

void execute_sequence_action(int32_t angle) {

  if (servo_state == IDLE) {

    persistent_angle = angle;

    digitalWrite(LED_PIN, LOW);

    servo_state = MOVING_TO_TARGET;

    sequence_start_time = millis();

    s1.write(180 - persistent_angle);
  }
}

// ======================================================
// CALLBACKS
// ======================================================

void callback_sequence(const void* msgin) {

  const std_msgs__msg__Int32* incoming_msg =
      (const std_msgs__msg__Int32*)msgin;

  if (incoming_msg->data > 0) {
    execute_sequence_action(incoming_msg->data);
  }
}

void callback_hold(const void* msgin) {

  const std_msgs__msg__Int32* incoming_msg =
      (const std_msgs__msg__Int32*)msgin;

  servo_state = IDLE;

  persistent_angle = incoming_msg->data;

  s1.write(180 - persistent_angle);
}

// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  randomSeed(micros());

  pinMode(INPUT_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  delay(250);
  digitalWrite(LED_PIN, LOW);
  delay(250);
  digitalWrite(LED_PIN, HIGH);
  delay(250);

  // ---------------- SERVO ----------------

  s1.attach(SERVO_PIN);

  s1.write(180);

  // ---------------- WIFI ----------------

  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  set_microros_wifi_transports(
      (char*)WIFI_SSID,
      (char*)WIFI_PASS,
      (char*)AGENT_IP,
      AGENT_PORT);

  Serial.println("Boot complete");
}

// ======================================================
// LOOP
// ======================================================

void loop() {

  // ==================================================
  // WIFI RECOVERY
  // ==================================================

  if (WiFi.status() != WL_CONNECTED) {

    if (device_state == AGENT_CONNECTED) {
      destroy_entities();
    }

    device_state = WAITING_WIFI;

    Serial.println("WiFi disconnected");

    WiFi.disconnect();

    delay(500);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    delay(1000);

    return;
  }

  // ==================================================
  // CONNECTION MANAGEMENT
  // ==================================================

  static uint32_t last_ping = 0;

  switch (device_state) {

    case WAITING_WIFI:

      if (WiFi.status() == WL_CONNECTED) {

        Serial.println("WiFi connected");

        device_state = AGENT_DISCONNECTED;
      }

      break;

    case AGENT_DISCONNECTED:

      if (millis() - last_ping > 1000) {

        last_ping = millis();

        Serial.println("Pinging agent...");

        if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK) {

          Serial.println("Agent reachable");

          if (create_entities()) {

            device_state = AGENT_CONNECTED;

            Serial.println("Connected to agent");
          }
        }
      }

      break;

    case AGENT_CONNECTED:

      if (millis() - last_ping > 2000) {

        last_ping = millis();

        if (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {

          Serial.println("Agent disconnected");

          destroy_entities();

          device_state = AGENT_DISCONNECTED;

          break;
        }
      }

      rclc_executor_spin_some(
          &executor,
          RCL_MS_TO_NS(20));

      break;
  }

  // ==================================================
  // BUTTON LOGIC
  // ==================================================

  bool current_pin_state = digitalRead(INPUT_PIN);

  if (current_pin_state == LOW &&
      last_pin_state == HIGH) {

    execute_sequence_action(persistent_angle);
  }

  last_pin_state = current_pin_state;

  // ==================================================
  // SERVO TIMING
  // ==================================================

  if (servo_state == MOVING_TO_TARGET &&
      millis() - sequence_start_time >= 500) {

    s1.write(180);

    servo_state = RETURNING_TO_180;

    sequence_start_time = millis();
  }

  else if (servo_state == RETURNING_TO_180 &&
           millis() - sequence_start_time >= 500) {

    servo_state = IDLE;

    digitalWrite(LED_PIN, HIGH);
  }

  // ==================================================
  // HEAP MONITORING
  // ==================================================

  static uint32_t last_heap_print = 0;

  if (millis() - last_heap_print > 5000) {

    last_heap_print = millis();

    Serial.printf(
        "[HEAP] Free: %u | Min: %u\n",
        ESP.getFreeHeap(),
        ESP.getMinFreeHeap());
  }

  // ==================================================
  // YIELD
  // ==================================================

  vTaskDelay(1);
}
