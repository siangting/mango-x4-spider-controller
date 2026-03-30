#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

// =======================================================
// Servo settings
// =======================================================
#define NUM_SERVOS 18

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
#define CMD_LOAD 0x1F

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
    100, 0  
  };
  sendPack(id, CMD_MOVE, p, 4);
}

// =======================================================
// micro-ROS variables
// =======================================================
rcl_subscription_t subscription;
rclc_executor_t    executor;
rcl_node_t         node;
rclc_support_t     support;
rcl_allocator_t    allocator;
trajectory_msgs__msg__JointTrajectory traj_msg;

bool agent_ok = false;

// =======================================================
// micro-ROS Callback
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
    sprintf(tmp, "servo_%u", i + 1);
    rosidl_runtime_c__String__assign(&traj_msg.joint_names.data[i], tmp);
  }

  trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(&traj_msg.points, 1);
  traj_msg.points.data[0].positions.data =
      (double*) malloc(NUM_SERVOS * sizeof(double));
  traj_msg.points.data[0].positions.size =
      traj_msg.points.data[0].positions.capacity = NUM_SERVOS;

  rclc_subscription_init_default(
    &subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    "/servo_trajectory"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(
        &executor, &subscription,
        &traj_msg, traj_callback,
        ON_NEW_DATA);

  return true;
}

void destroy_entities() {
  rclc_executor_fini(&executor);
  rcl_subscription_fini(&subscription, &node);
  rcl_node_fini(&node);
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