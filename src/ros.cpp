#include "ros.h"

states state;

rcl_publisher_t publisher;
rcl_publisher_t publisher_pose;
rcl_publisher_t publisher_batt;
std_msgs__msg__Bool msg;
std_msgs__msg__Float32 msg_batt;
geometry_msgs__msg__PoseStamped msg_pose;
geometry_msgs__msg__PoseStamped received_msg;
geometry_msgs__msg__PoseStamped received_msg_fix_pose;
std_msgs__msg__Bool received_msg_noisette;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_subscription_t subscriber_noisette;
rcl_subscription_t subscriber_fix_pose;
rclc_executor_t executor;

void init_ros() {
  Serial.printf("Connecting to ap: %s\n", ENV_WIFI_SSID);
  IPAddress agent_ip(ENV_AGENT_IP);
  uint16_t agent_port = 8888;
  set_microros_wifi_transports(ENV_WIFI_SSID, ENV_WIFI_PASSWORD, agent_ip, agent_port);
  state = states::WAITING_AGENT;
}

bool ros_update_odometry() {
  msg_pose.header.stamp.sec = 0;
  msg_pose.header.stamp.nanosec = 0;
  rosidl_runtime_c__String__assign(&msg_pose.header.frame_id, "world");
  msg_pose.pose.position.x = odometry.current.x;
  msg_pose.pose.position.y = odometry.current.y;
  msg_pose.pose.position.z = 0;
  msg_pose.pose.orientation.x = 0;
  msg_pose.pose.orientation.y = 0;
  double half_theta = (odometry.current.theta) * 0.5;
  msg_pose.pose.orientation.z = sin(half_theta);
  msg_pose.pose.orientation.w = cos(half_theta);
  RCCHECK(rcl_publish(&publisher_pose, &msg_pose, NULL));
  return true;
}

bool ros_update_obstacle() {
  msg.data = obstacle_detected;
  RCCHECK(rcl_publish(&publisher, &msg, NULL));
  return true;
}

bool ros_update_batt() {
  msg_batt.data = analogReadMilliVolts(VSENSE_PIN)/3*(1800+10000)/1800;
  RCCHECK(rcl_publish(&publisher_batt, &msg_batt, NULL));
  return true;
}

void NoisetteCallback(const void* msgin) {
  const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
  noisette = msg->data;
}

void FixPoseCallback(const void* msgin) {
  const geometry_msgs__msg__PoseStamped* msg = (const geometry_msgs__msg__PoseStamped*)msgin;
  Serial.println("Setting current from ROS");
  odometry.current.theta = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  odometry.current.x = msg->pose.position.x;
  odometry.current.y = msg->pose.position.y;
  target_pos.theta = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  target_pos.x = msg->pose.position.x;
  target_pos.y = msg->pose.position.y;
  if (motors_state != motors_state_t::WAITING) {
    new_pose = true;
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, ENV_NAMESPACE, "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    ENV_NAMESPACE"/obstacle"));
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_pose,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
    ENV_NAMESPACE"/position"));
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_batt,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    ENV_NAMESPACE"/batt"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
    "/goal_pose"));
  RCCHECK(rclc_subscription_init_default(&subscriber_noisette, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    ENV_NAMESPACE"/noisette"));
  RCCHECK(rclc_subscription_init_default(&subscriber_fix_pose, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
    ENV_NAMESPACE"/fix_pose"));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &received_msg,
      &SubscriptionCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_noisette, &received_msg_noisette,
      &NoisetteCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_fix_pose, &received_msg_fix_pose,
      &FixPoseCallback, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void) rcl_publisher_fini(&publisher, &node);
  (void) rcl_publisher_fini(&publisher_pose, &node);
  (void) rcl_publisher_fini(&publisher_batt, &node);
  (void) rclc_executor_fini(&executor);
  (void) rcl_subscription_fini(&subscriber, &node);
  (void) rcl_subscription_fini(&subscriber_noisette, &node);
  (void) rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void ros_loop() {
  switch (state) {
  case states::WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? states::AGENT_AVAILABLE
                                        : states::WAITING_AGENT;);
    break;
  case states::AGENT_AVAILABLE:
    state = (true == create_entities()) ? states::AGENT_CONNECTED : states::WAITING_AGENT;
    if (state == states::WAITING_AGENT) {
      destroy_entities();
    };
    break;
  case states::AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? states::AGENT_CONNECTED
                                        : states::AGENT_DISCONNECTED;);
    if (state == states::AGENT_CONNECTED) {
        EXECUTE_EVERY_N_MS(100, ros_update_obstacle());
        EXECUTE_EVERY_N_MS(100, ros_update_odometry());
        EXECUTE_EVERY_N_MS(1000, ros_update_batt());
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;
  case states::AGENT_DISCONNECTED:
    destroy_entities();
    state = states::WAITING_AGENT;
    break;
  default:
    break;
  }
}

void SubscriptionCallback(const void* msgin) {
  const geometry_msgs__msg__PoseStamped* msg = (const geometry_msgs__msg__PoseStamped*)msgin;
  Serial.println("Setting target from ROS");
  target_pos.theta = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  target_pos.x = msg->pose.position.x;
  target_pos.y = msg->pose.position.y;
  if (motors_state != motors_state_t::WAITING) {
    new_pose = true;
  }
}