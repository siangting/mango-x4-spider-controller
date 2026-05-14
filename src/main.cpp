#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <string.h>

#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

// =======================================================
// Servo settings
// =======================================================
#define NUM_SERVOS 18
#define SERVO_ID_MIN 0
#define SERVO_ID_MAX 255

#define SERVO_STATUS_PERIOD_MS 100
#define SERVO_STATUS_SCAN_IDS_PER_TICK 8
#define SERVO_READ_TIMEOUT_MS 6
#define SERVO_SCAN_STEP_DELAY_MS 1
#define SERVO_MOVE_TIME_MS 100

// ======= RP2040 / Pico UART1 =======
// Valid pairs: TX=4/RX=5, TX=8/RX=9, TX=20/RX=21
#define TX_PIN 8
#define RX_PIN 9

#define BusSerial Serial2

// =======================================================
// Protocol (LewanSoul / STS / HTD-45H compatible)
// =======================================================
#define HDR      0x55
#define CMD_MOVE 0x01
#define CMD_ID_WRITE 0x0D
#define CMD_POS_READ 0x1C
#define CMD_LOAD 0x1F

// =======================================================
// Servo protocol helpers
// =======================================================

uint8_t calcCHK(const uint8_t *b) {
  uint16_t s = 0;
  for (uint8_t i = 2; i < b[3] + 2; i++) s += b[i];
  return ~s;
}

void sendPack(uint8_t id, uint8_t cmd, const uint8_t *p, uint8_t n) {
  uint8_t buf[6 + 16];
  buf[0] = buf[1] = HDR;
  buf[2] = id;
  buf[3] = n + 3;
  buf[4] = cmd;
  for (uint8_t i = 0; i < n; i++) buf[5 + i] = p[i];
  buf[5 + n] = calcCHK(buf);
  BusSerial.write(buf, 6 + n);
}

void clearBusRx() {
  while (BusSerial.available() > 0) {
    BusSerial.read();
  }
}

bool readPacket(uint8_t expected_id, uint8_t expected_cmd,
                uint8_t *params, uint8_t &param_len,
                uint16_t timeout_ms) {
  uint8_t buf[16];
  uint8_t idx = 0;
  uint32_t start = millis();

  while ((millis() - start) < timeout_ms && idx < sizeof(buf)) {
    if (BusSerial.available() <= 0) {
      delayMicroseconds(100);
      continue;
    }

    uint8_t b = BusSerial.read();

    if (idx == 0 && b != HDR) continue;
    if (idx == 1 && b != HDR) {
      idx = (b == HDR) ? 1 : 0;
      continue;
    }

    buf[idx++] = b;

    if (idx >= 4) {
      uint8_t packet_len = buf[3];
      uint8_t total_len = packet_len + 3;
      if (total_len > sizeof(buf)) return false;
      if (idx >= total_len) {
        if (buf[2] != expected_id || buf[4] != expected_cmd) return false;
        if (calcCHK(buf) != buf[total_len - 1]) return false;

        param_len = packet_len - 3;
        for (uint8_t i = 0; i < param_len; i++) {
          params[i] = buf[5 + i];
        }
        return true;
      }
    }
  }

  return false;
}

void enableTorque(uint8_t id) {
  uint8_t on = 1;
  sendPack(id, CMD_LOAD, &on, 1);
}

void moveServoDeg(uint8_t id, float deg) {
  deg = constrain(deg, 0.0f, 240.0f);
  uint16_t pos = (uint16_t)(deg / 240.0f * 1000.0f);

  uint8_t p[4] = {
    (uint8_t)(pos & 0xFF),
    (uint8_t)(pos >> 8),
    (uint8_t)(SERVO_MOVE_TIME_MS & 0xFF),
    (uint8_t)(SERVO_MOVE_TIME_MS >> 8)
  };
  sendPack(id, CMD_MOVE, p, 4);
}

bool readServoAngle(uint8_t id, float &angle) {
  uint8_t params[4];
  uint8_t param_len = 0;

  clearBusRx();
  sendPack(id, CMD_POS_READ, nullptr, 0);
  BusSerial.flush();

  if (!readPacket(id, CMD_POS_READ, params, param_len, SERVO_READ_TIMEOUT_MS) ||
      param_len < 2) {
    return false;
  }

  int16_t pos = (int16_t)(params[0] | (params[1] << 8));
  pos = constrain(pos, 0, 1000);
  angle = (float)pos * 240.0f / 1000.0f;
  return true;
}

bool scanServo(uint8_t &found_id, float *angle = nullptr, uint16_t max_checks = 256) {
  static uint16_t next_id = SERVO_ID_MIN;

  for (uint16_t scanned = 0; scanned < max_checks; scanned++) {
    uint8_t id = (uint8_t)next_id;
    float current_angle = 0.0f;

    next_id = (next_id >= SERVO_ID_MAX) ? SERVO_ID_MIN : next_id + 1;

    if (readServoAngle(id, current_angle)) {
      found_id = id;
      if (angle != nullptr) {
        *angle = current_angle;
      }
      return true;
    }

    delay(SERVO_SCAN_STEP_DELAY_MS);
  }

  return false;
}

bool setServoID(uint8_t current_id, uint8_t new_id) {
  uint8_t p[1] = {new_id};

  Serial.printf("[servo_config] Changing servo ID %u -> %u\n", current_id, new_id);
  sendPack(current_id, CMD_ID_WRITE, p, 1);
  BusSerial.flush();
  delay(30);

  float angle = 0.0f;
  bool ok = readServoAngle(new_id, angle);
  Serial.printf("[servo_config] ID change %s, verify ID %u%s\n",
                ok ? "OK" : "not verified",
                new_id,
                ok ? "" : " failed");
  return ok;
}

// =======================================================
// micro-ROS variables
// =======================================================
rcl_subscription_t traj_subscription;
rcl_subscription_t config_subscription;
rcl_subscription_t test_subscription;
rcl_publisher_t    status_publisher;
rclc_executor_t    executor;
rcl_node_t         node;
rclc_support_t     support;
rcl_allocator_t    allocator;
trajectory_msgs__msg__JointTrajectory traj_msg;
std_msgs__msg__String config_msg;
std_msgs__msg__String test_msg;
std_msgs__msg__String status_msg;

bool agent_ok = false;
uint32_t last_status_ms = 0;

char config_msg_buffer[32];
char test_msg_buffer[32];
char status_msg_buffer[48];

// =======================================================
// micro-ROS callbacks and publishers
// =======================================================
void traj_callback(const void * msgin) {
  auto *t = (const trajectory_msgs__msg__JointTrajectory *)msgin;
  if (t->points.size == 0) return;

  const auto &pt = t->points.data[0];

  size_t n = pt.positions.size;
  if (n > NUM_SERVOS) n = NUM_SERVOS;

  for (size_t i = 0; i < n; i++) {
    float deg = (float) pt.positions.data[i];
    moveServoDeg(i + 1, deg);
  }
}

bool parseServoCommand(const std_msgs__msg__String *msg, uint8_t &id, float &angle) {
  char command[32];
  int parsed_id = -1;
  float parsed_angle = 0.0f;
  size_t len = msg->data.size;

  if (len >= sizeof(command)) {
    len = sizeof(command) - 1;
  }
  memcpy(command, msg->data.data, len);
  command[len] = '\0';

  if (sscanf(command, "%d %f", &parsed_id, &parsed_angle) != 2) {
    return false;
  }
  if (parsed_id < SERVO_ID_MIN || parsed_id > SERVO_ID_MAX) {
    return false;
  }

  id = (uint8_t)parsed_id;
  angle = constrain(parsed_angle, 0.0f, 240.0f);
  return true;
}

void config_callback(const void * msgin) {
  const auto *msg = (const std_msgs__msg__String *)msgin;
  uint8_t new_id = 0;
  float initial_angle = 0.0f;

  if (!parseServoCommand(msg, new_id, initial_angle)) {
    Serial.println("[servo_config] Invalid message, expected: 'new_id angle'");
    return;
  }

  Serial.printf("[servo_config] Request new_id=%u angle=%.1f\n", new_id, initial_angle);

  uint8_t current_id = 0;
  float current_angle = 0.0f;
  if (!scanServo(current_id, &current_angle, 256)) {
    Serial.println("[servo_config] No servo detected while scanning IDs 0-255");
    return;
  }

  Serial.printf("[servo_config] Detected servo ID %u angle %.1f\n", current_id, current_angle);

  if (current_id != new_id) {
    setServoID(current_id, new_id);
  }

  enableTorque(new_id);
  moveServoDeg(new_id, initial_angle);
  Serial.printf("[servo_config] Servo %u moved to %.1f deg\n", new_id, initial_angle);
}

void test_callback(const void * msgin) {
  const auto *msg = (const std_msgs__msg__String *)msgin;
  uint8_t servo_id = 0;
  float angle = 0.0f;

  if (!parseServoCommand(msg, servo_id, angle)) {
    Serial.println("[servo_test] Invalid message, expected: 'servo_id angle'");
    return;
  }

  Serial.printf("[servo_test] Moving servo %u to %.1f deg\n", servo_id, angle);
  enableTorque(servo_id);
  moveServoDeg(servo_id, angle);
}

void publishServoStatus() {
  if (!agent_ok || (millis() - last_status_ms) < SERVO_STATUS_PERIOD_MS) {
    return;
  }
  last_status_ms = millis();

  uint8_t id = 0;
  float angle = 0.0f;
  if (!scanServo(id, &angle, SERVO_STATUS_SCAN_IDS_PER_TICK)) {
    return;
  }

  snprintf(status_msg_buffer, sizeof(status_msg_buffer),
           "id:%u angle:%d", id, (int)(angle + 0.5f));
  status_msg.data.size = strlen(status_msg_buffer);

  if (rcl_publish(&status_publisher, &status_msg, NULL) == RCL_RET_OK) {
    Serial.printf("[servo_status] Published %s\n", status_msg_buffer);
  } else {
    Serial.println("[servo_status] Publish failed");
  }
}

// =======================================================
// micro-ROS entity creation
// =======================================================
bool create_entities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    return false;

  if (rclc_node_init_default(&node, "servo_node", "", &support) != RCL_RET_OK)
    return false;

  trajectory_msgs__msg__JointTrajectory__init(&traj_msg);

  rosidl_runtime_c__String__Sequence__init(&traj_msg.joint_names, NUM_SERVOS);
  char tmp[16];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    sprintf(tmp, "joint%u", i + 1);
    rosidl_runtime_c__String__assign(&traj_msg.joint_names.data[i], tmp);
  }

  trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(&traj_msg.points, 1);
  traj_msg.points.data[0].positions.data =
      (double*) malloc(NUM_SERVOS * sizeof(double));
  traj_msg.points.data[0].positions.size =
      traj_msg.points.data[0].positions.capacity = NUM_SERVOS;

  rclc_subscription_init_default(
    &traj_subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    "/servo_trajectory"
  );

  std_msgs__msg__String__init(&config_msg);
  config_msg.data.data = config_msg_buffer;
  config_msg.data.size = 0;
  config_msg.data.capacity = sizeof(config_msg_buffer);

  std_msgs__msg__String__init(&test_msg);
  test_msg.data.data = test_msg_buffer;
  test_msg.data.size = 0;
  test_msg.data.capacity = sizeof(test_msg_buffer);

  std_msgs__msg__String__init(&status_msg);
  status_msg.data.data = status_msg_buffer;
  status_msg.data.size = 0;
  status_msg.data.capacity = sizeof(status_msg_buffer);

  rclc_subscription_init_default(
    &config_subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/servo_config"
  );

  rclc_subscription_init_default(
    &test_subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/servo_test"
  );

  rclc_publisher_init_default(
    &status_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/servo_status"
  );

  rclc_executor_init(&executor, &support.context, 3, &allocator);

  rclc_executor_add_subscription(
        &executor, &traj_subscription,
        &traj_msg, traj_callback,
        ON_NEW_DATA);

  rclc_executor_add_subscription(
        &executor, &config_subscription,
        &config_msg, config_callback,
        ON_NEW_DATA);

  rclc_executor_add_subscription(
        &executor, &test_subscription,
        &test_msg, test_callback,
        ON_NEW_DATA);

  return true;
}

void destroy_entities() {
  rclc_executor_fini(&executor);
  rcl_ret_t ret = RCL_RET_OK;
  ret = rcl_publisher_fini(&status_publisher, &node);
  ret = rcl_subscription_fini(&test_subscription, &node);
  ret = rcl_subscription_fini(&config_subscription, &node);
  ret = rcl_subscription_fini(&traj_subscription, &node);
  ret = rcl_node_fini(&node);
  (void)ret;
  rclc_support_fini(&support);
}

// =======================================================
// micro-ROS with auto-reconnect
// =======================================================
void micro_ros_loop() {
  if (!agent_ok) {
    if (rmw_uros_ping_agent(100, 2) == RMW_RET_OK) {
      Serial.println("[micro-ROS] Agent detected. Creating entities...");
      if (create_entities()) {
        agent_ok = true;
        Serial.println("[micro-ROS] Connected!");
      }
    }
    return;
  }

  // Connected -> execute executor
  if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {
    Serial.println("[micro-ROS] Lost connection, resetting...");
    destroy_entities();
    agent_ok = false;
    delay(200);
  }

  publishServoStatus();
}

// =======================================================
// setup()
// =======================================================
void setup() {

  // Wait for USB enumeration to avoid micro-ROS initialization disconnecting too early
  delay(800);

  Serial.begin(115200);
  delay(200);

  // ===== UART1 (BusServo) =====
  BusSerial.setRX(RX_PIN);
  BusSerial.setTX(TX_PIN);
  BusSerial.begin(115200, SERIAL_8N1);
  delay(50);

  for (uint8_t id = 1; id <= NUM_SERVOS; id++) {
    enableTorque(id);
    delay(10);
  }

  // ===== micro-ROS over USB CDC =====
  set_microros_serial_transports(Serial);

  Serial.println("Boot complete. Waiting for micro-ROS agent...");
}

// =======================================================
// loop()
// =======================================================
void loop() {
  micro_ros_loop();
}
