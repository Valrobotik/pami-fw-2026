#include "ros.h"

rcl_publisher_t publisher;
rcl_publisher_t publisher_pose;
std_msgs__msg__Bool msg;
geometry_msgs__msg__PoseStamped msg_pose;
geometry_msgs__msg__PoseStamped received_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;

void init_ros() {
  Serial.printf("Connecting to ap: %s\n", ENV_WIFI_SSID);
  IPAddress agent_ip(ENV_AGENT_IP);
  uint16_t agent_port = 8888;
  set_microros_wifi_transports(ENV_WIFI_SSID, ENV_WIFI_PASSWORD, agent_ip, agent_port);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_wifi_node", "", &support));
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
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
     "/goal_pose"));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &received_msg,
      &SubscriptionCallback, ON_NEW_DATA));
}

void ros_update_odometry() {
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
}

void ros_update_obstacle() {
  msg.data = obstacle_detected;
  RCCHECK(rcl_publish(&publisher, &msg, NULL));
}

void RosTask(void *pvParams) {
  while (1) {
    EXECUTE_EVERY_N_MS(100, ros_update_obstacle());
    EXECUTE_EVERY_N_MS(100, ros_update_odometry());
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    delay(20);
  }
}

void SubscriptionCallback(const void* msgin) {
  const geometry_msgs__msg__PoseStamped* msg = (const geometry_msgs__msg__PoseStamped*)msgin;
  Serial.println("Setting target from ROS");
  target_pos.theta = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  target_pos.x = msg->pose.position.x;
  target_pos.y = msg->pose.position.y;
}