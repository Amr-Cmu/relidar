#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>
#include <EasyUltrasonic.h>

#define LED_PIN 13
#define TRIGPIN_LEFT  32
#define ECHOPIN_LEFT  25
#define TRIGPIN_RIGHT 26
#define ECHOPIN_RIGHT 27

#define DISTANCE_THRESHOLD 70  
#define ALPHA 0.5  // ค่า LPF (0.1 - 0.3 ค่าต่ำ = กรองมาก ค่าสูง = ตอบสนองเร็ว)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t pub_left;
rcl_publisher_t pub_right;
std_msgs__msg__Float32 msg_left;
std_msgs__msg__Float32 msg_right;
bool micro_ros_init_successful;

EasyUltrasonic ultrasonicL;
EasyUltrasonic ultrasonicR;

// ตัวแปรค่าเซนเซอร์ที่กรองแล้ว
float filtered_distanceL = DISTANCE_THRESHOLD;
float filtered_distanceR = DISTANCE_THRESHOLD;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    // อ่านค่าจากเซนเซอร์
    float raw_distanceL = ultrasonicL.getDistanceIN() * 2.54;
    float raw_distanceR = ultrasonicR.getDistanceIN() * 2.54;

    // กรองค่าด้วย Exponential Moving Average (EMA)
    filtered_distanceL = (ALPHA * raw_distanceL) + ((1 - ALPHA) * filtered_distanceL);
    filtered_distanceR = (ALPHA * raw_distanceR) + ((1 - ALPHA) * filtered_distanceR);

    // ตั้งค่าข้อมูลที่ส่งไป ROS 2
    msg_left.data = filtered_distanceL;
    msg_right.data = filtered_distanceR;

    // แสดงค่าผ่าน Serial Monitor
    Serial.print("Filtered Left: ");
    Serial.print(filtered_distanceL);
    Serial.println(" cm");

    Serial.print("Filtered Right: ");
    Serial.print(filtered_distanceR);
    Serial.println(" cm");

    // ส่งค่าผ่าน ROS 2
    rcl_publish(&pub_left, &msg_left, NULL);
    rcl_publish(&pub_right, &msg_right, NULL);
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "ultrasonic_publisher_rclc", "", &support));

  // create publishers
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "Ultra_state1"));

  RCCHECK(rclc_publisher_init_best_effort(
    &pub_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "Ultra_state2"));

  // create timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub_left, &node);
  rcl_publisher_fini(&pub_right, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);  
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  ultrasonicL.attach(TRIGPIN_LEFT, ECHOPIN_LEFT);
  ultrasonicR.attach(TRIGPIN_RIGHT, ECHOPIN_RIGHT);

  state = WAITING_AGENT;

  msg_left.data = 0.0;
  msg_right.data = 0.0;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
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

  digitalWrite(LED_PIN, state == AGENT_CONNECTED);
}
