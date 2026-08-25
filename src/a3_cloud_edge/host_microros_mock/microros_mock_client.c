#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/joint_state.h"
#include "trajectory_msgs/msg/joint_trajectory.h"
#include "micro_ros_utilities/type_utilities.h"

#define NUM_JOINTS 7
#define MAX_POINTS 32
#define MAX_JOINT_NAME_LEN 32

#define RCCHECK(fn) \
  do { \
    rcl_ret_t temp_rc = (fn); \
    if (temp_rc != RCL_RET_OK) { \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      return 1; \
    } \
  } while (0)

typedef struct {
  size_t count;
  double positions[MAX_POINTS][NUM_JOINTS];
  double times[MAX_POINTS];
} TrajectoryBuffer;

static TrajectoryBuffer g_traj;
static bool g_has_traj;
static rcl_time_point_value_t g_traj_start_ns;
static rcl_clock_t * g_clock;
static rcl_publisher_t * g_pub;
static sensor_msgs__msg__JointState * g_js;

static const char * k_joint_names[NUM_JOINTS] = {
  "L1_joint", "L2_joint", "L3_joint", "L4_joint",
  "L5_joint", "L6_joint", "L7_joint"};

static double duration_to_sec(const builtin_interfaces__msg__Duration * d)
{
  return (double)d->sec + (double)d->nanosec * 1e-9;
}

static void sample_at_time(double elapsed)
{
  if (!g_has_traj || g_traj.count == 0) {
    return;
  }

  if (elapsed >= g_traj.times[g_traj.count - 1]) {
    memcpy(g_js->position.data, g_traj.positions[g_traj.count - 1], sizeof(double) * NUM_JOINTS);
    g_has_traj = false;
    return;
  }

  size_t idx = 0;
  while (idx + 1 < g_traj.count && elapsed > g_traj.times[idx + 1]) {
    ++idx;
  }

  const double t0 = g_traj.times[idx];
  const double t1 = g_traj.times[idx + 1];
  const double alpha = (t1 > t0) ? (elapsed - t0) / (t1 - t0) : 0.0;

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    g_js->position.data[i] =
      g_traj.positions[idx][i] + alpha * (g_traj.positions[idx + 1][i] - g_traj.positions[idx][i]);
  }
}

static void traj_callback(const void * msgin)
{
  const trajectory_msgs__msg__JointTrajectory * msg =
    (const trajectory_msgs__msg__JointTrajectory *)msgin;
  if (msg->points.size == 0) {
    printf("[mock] empty trajectory ignored\n");
    return;
  }
  if (msg->points.size > MAX_POINTS) {
    printf("[mock] trajectory too long (%zu > %d), truncating\n",
      msg->points.size, MAX_POINTS);
  }

  g_traj.count = msg->points.size < MAX_POINTS ? msg->points.size : MAX_POINTS;
  for (size_t p = 0; p < g_traj.count; ++p) {
    g_traj.times[p] = duration_to_sec(&msg->points.data[p].time_from_start);
    for (size_t j = 0; j < NUM_JOINTS; ++j) {
      if (j < msg->points.data[p].positions.size) {
        g_traj.positions[p][j] = msg->points.data[p].positions.data[j];
      } else {
        g_traj.positions[p][j] = 0.0;
      }
    }
  }

  rcl_clock_get_now(g_clock, &g_traj_start_ns);
  g_has_traj = true;
  printf(
    "[mock] trajectory received: %zu points, duration=%.3fs, "
    "start=[%.4f %.4f %.4f] end=[%.4f %.4f %.4f]\n",
    g_traj.count,
    g_traj.times[g_traj.count - 1],
    g_traj.positions[0][0], g_traj.positions[0][1], g_traj.positions[0][2],
    g_traj.positions[g_traj.count - 1][0],
    g_traj.positions[g_traj.count - 1][1],
    g_traj.positions[g_traj.count - 1][2]);
  fflush(stdout);
}

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void)timer;
  (void)last_call_time;

  rcl_time_point_value_t now_ns = 0;
  rcl_clock_get_now(g_clock, &now_ns);
  if (g_has_traj) {
    sample_at_time((now_ns - g_traj_start_ns) * 1e-9);
  }
  g_js->header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
  g_js->header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

  (void)rcl_publish(g_pub, g_js, NULL);
}

static bool allocate_trajectory_msg(trajectory_msgs__msg__JointTrajectory * msg)
{
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory);

  micro_ros_utilities_memory_rule_t rules[] = {
    {"joint_names", NUM_JOINTS},
    {"points", MAX_POINTS},
    {"points.positions", NUM_JOINTS},
    {"points.velocities", NUM_JOINTS},
    {"points.accelerations", NUM_JOINTS},
    {"points.effort", NUM_JOINTS},
  };

  micro_ros_utilities_memory_conf_t conf = micro_ros_utilities_memory_conf_default;
  conf.max_string_capacity = MAX_JOINT_NAME_LEN;
  conf.max_ros2_type_sequence_capacity = MAX_POINTS;
  conf.max_basic_type_sequence_capacity = NUM_JOINTS;
  conf.rules = rules;
  conf.n_rules = sizeof(rules) / sizeof(rules[0]);

  if (!micro_ros_utilities_create_message_memory(type_support, msg, conf)) {
    printf("[mock] create_message_memory failed\n");
    return false;
  }
  return true;
}

int main(int argc, const char * const * argv)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();

  const char * agent_ip = getenv("XRCE_AGENT_IP");
  const char * agent_port = getenv("XRCE_AGENT_PORT");
  if (agent_ip == NULL || agent_ip[0] == '\0') {
    agent_ip = "127.0.0.1";
  }
  if (agent_port == NULL || agent_port[0] == '\0') {
    agent_port = "8888";
  }

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  if (rmw_uros_options_set_udp_address(agent_ip, agent_port, rmw_options) != RMW_RET_OK) {
    printf("[mock] rmw_uros_options_set_udp_address failed (%s:%s)\n", agent_ip, agent_port);
    return 1;
  }
  printf("[mock] Agent %s:%s\n", agent_ip, agent_port);
  fflush(stdout);

  rclc_support_t support;
  RCCHECK(rclc_support_init_with_options(&support, argc, argv, &init_options, &allocator));
  g_clock = &support.clock;

  {
    int ping_tries = 0;
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
      ++ping_tries;
      if (ping_tries > 30) {
        printf("[mock] Agent ping timeout\n");
        return 1;
      }
      printf("[mock] waiting for Agent (%d)\n", ping_tries);
      fflush(stdout);
    }
    printf("[mock] Agent ping ok\n");
    fflush(stdout);
  }

  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "microros_mock_client", "", &support));

  rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
  RCCHECK(rclc_subscription_init_default(
    &sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    "/joint_group_effort_controller/joint_trajectory"));

  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  RCCHECK(rclc_publisher_init_default(
    &pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));
  g_pub = &pub;

  sensor_msgs__msg__JointState js;
  if (!sensor_msgs__msg__JointState__init(&js)) {
    return 1;
  }
  js.name.capacity = NUM_JOINTS;
  js.name.size = NUM_JOINTS;
  js.name.data = (rosidl_runtime_c__String *)allocator.allocate(
    NUM_JOINTS * sizeof(rosidl_runtime_c__String), allocator.state);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    rosidl_runtime_c__String__init(&js.name.data[i]);
    rosidl_runtime_c__String__assign(&js.name.data[i], k_joint_names[i]);
  }
  js.position.capacity = NUM_JOINTS;
  js.position.size = NUM_JOINTS;
  js.position.data = (double *)allocator.allocate(NUM_JOINTS * sizeof(double), allocator.state);
  memset(js.position.data, 0, NUM_JOINTS * sizeof(double));
  g_js = &js;

  trajectory_msgs__msg__JointTrajectory traj_msg;
  if (!allocate_trajectory_msg(&traj_msg)) {
    return 1;
  }

  rcl_timer_t timer;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));

  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &sub, &traj_msg, &traj_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  printf("[mock] ready: sub=/joint_group_effort_controller/joint_trajectory pub=/joint_states\n");
  fflush(stdout);

  while (rcl_context_is_valid(&support.context)) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  }

  rclc_executor_fini(&executor);
  (void)rcl_subscription_fini(&sub, &node);
  (void)rcl_publisher_fini(&pub, &node);
  (void)rcl_node_fini(&node);
  rclc_support_fini(&support);
  return 0;
}
