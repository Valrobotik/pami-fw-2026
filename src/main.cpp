#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include "Commander.h"
#include <FastLED.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <rosidl_runtime_c/string_functions.h>


#include "pin_definitions.h"
#include "motor.h"

#define DIAMETRE_ROUE 65// en millimètre
#define LARGEUR 123// en millimètre largeur entre les 2 roues
#define STEPS_PER_ROT 51200
#define DISTANCE 100 // en millimetre, distance avant le mur
#define LONGUEURCAPLA 100 // en millimetre, longueur d'un obstacle

// ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("Erreur ROS");}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0) \

rcl_publisher_t publisher;
rcl_publisher_t publisher_pose;
std_msgs__msg__Bool msg;
geometry_msgs__msg__PoseStamped msg_pose;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
 
Commander command = Commander(Serial);

extern FastAccelStepperEngine engine;
Stepper stepper1 = Stepper(DIR_1_PIN, STEP_1_PIN, SLEEP_1_PIN, VREF_1_PIN, EN_1_PIN, 300);
Stepper stepper2 = Stepper(DIR_2_PIN, STEP_2_PIN, SLEEP_2_PIN, VREF_2_PIN, EN_2_PIN, 300);

CRGB led[1];

typedef struct pose_2d_t {
  float x;
  float y;
  float theta;
} pose_2d_t;

typedef struct odometry_status_t {
  pose_2d_t current;
  int32_t last_stepper_1;
  int32_t last_stepper_2;
  unsigned long last_time;
} odometry_status_t;

odometry_status_t odometry = {0};
pose_2d_t target_pos = {0};

typedef enum motors_state_t {
  OFF,
  TURNING,
  FORWARD,
  WAITING,
  STOPPING_TURNING,
  STOPPING_FORWARD,
} motors_state_t;
motors_state_t motors_state = motors_state_t::OFF;

bool obstacle_detected = false;
void StopTask(void *pvParams) {
  Serial.println("Started stop task");
  while (1) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeMilliMeter < DISTANCE) {
      obstacle_detected = true;
    } else {
      obstacle_detected = false;
    }
    delay(50);
  }
}

void LightTask(void *pvParams) {
  Serial.println("Started light task");
  while (1) {
    switch (motors_state) {
    case motors_state_t::OFF:
      led[0] = CRGB::Red1;
      break;
    case motors_state_t::TURNING:
      led[0] = CRGB::Yellow2;
      break;
    case motors_state_t::FORWARD:
      led[0] = CRGB::Blue2;
      break;
    case motors_state_t::WAITING:
      led[0] = CRGB::Green;
      break;
    }
    if (obstacle_detected && (millis() % 500) < 100) {
      led[0] = CRGB::Red3;
    }
    FastLED.show();
    delay(50);
  }
}

// exprime theta entre -pi et pi
float clamp_angle(float theta) {
  if (theta > PI) {
    theta -= (2 * PI);
  } if (theta < -PI) {
    theta += (2 * PI);
  }
  return theta;
}

int distance_to_step(float d) { //d en millimètre distance à parcourir
  return (d*STEPS_PER_ROT)/(DIAMETRE_ROUE*PI);
}

float step_to_distance(int s) {
  return (s*DIAMETRE_ROUE*PI)/STEPS_PER_ROT;
}

void OdoTask(void *pvParams) {
  while (1) {
    int32_t d_step_1 = -(stepper1.getCurrentPosition() - odometry.last_stepper_1);
    odometry.last_stepper_1 = stepper1.getCurrentPosition();
    int32_t d_step_2 = stepper2.getCurrentPosition() - odometry.last_stepper_2;
    odometry.last_stepper_2 = stepper2.getCurrentPosition();
    float distance_moyenne = (step_to_distance(d_step_1) + step_to_distance(d_step_2))/2;
    float d_theta = (step_to_distance(d_step_2) - step_to_distance(d_step_1))/LARGEUR;
    float new_theta = odometry.current.theta + d_theta;
    odometry.current.theta = new_theta;
    odometry.current.theta = clamp_angle(odometry.current.theta);
    odometry.current.x += distance_moyenne*cos(new_theta)/1000;
    odometry.current.y += distance_moyenne*sin(new_theta)/1000;
    delay(10);
  }
}

void ros_update_odometry() {
  // msg_pose.header.stamp.sec = 0;
  // msg_pose.header.stamp.nanosec = 0;
  // rosidl_runtime_c__String__assign(&msg_pose.header.frame_id, "world");
  // msg_pose.pose.position.x = ((double) odometry_status.current_x)/1000;
  // msg_pose.pose.position.y = ((double)odometry_status.current_y)/1000;
  // msg_pose.pose.position.z = 0;
  // msg_pose.pose.orientation.x = 0;
  // msg_pose.pose.orientation.y = 0;
  // double half_theta = (odometry_status.current_angle * PI / 180.0) * 0.5;
  // msg_pose.pose.orientation.z = sin(half_theta);
  // msg_pose.pose.orientation.w = cos(half_theta);
  // RCCHECK(rcl_publish(&publisher_pose, &msg_pose, NULL));
}

void ros_update_obstacle() {
  msg.data = obstacle_detected;
  RCCHECK(rcl_publish(&publisher, &msg, NULL));
}

void RosTask(void *pvParams) {
  while (1) {
    EXECUTE_EVERY_N_MS(100, ros_update_obstacle());
    EXECUTE_EVERY_N_MS(100, ros_update_odometry());
    delay(20);
  }
}

void bras_noisette(){
  for (int pos = 0; pos < 256; pos++) {  // go from 0-180 degrees
    ledcWrite(BUZZER_PIN, pos);
    delay(10);
  }
  for (int pos = 255; pos >= 0; pos--) {  // go from 180-0 degrees
    ledcWrite(BUZZER_PIN, pos);
    delay(10);
  }
}

void avancer(int d){ // parametre en millimetre
  motors_state = motors_state_t::FORWARD;
  stepper1.move(-distance_to_step(d));
  stepper2.move(distance_to_step(d));
  while (stepper2.isRunning()&& stepper1.isRunning()){
    if (obstacle_detected) {
      stepper2.stopMove();
      stepper1.stopMove();
      motors_state = motors_state_t::STOPPING_FORWARD;
      delay(1000); // prsk c'est comme ca
    }
    delay(50);
  }
  motors_state = motors_state_t::WAITING;
}

void tourner(float rot){ // rot en radian [-pi;pi]
  motors_state = motors_state_t::TURNING;
  stepper2.move(distance_to_step(LARGEUR*rot/2));
  stepper1.move(-distance_to_step(-LARGEUR*rot/2));
  while (stepper2.isRunning()&& stepper1.isRunning()){
    if (obstacle_detected) {
      stepper2.stopMove();
      stepper1.stopMove();
      motors_state = motors_state_t::STOPPING_TURNING;
      delay(1000); // prsk c'est comme ca
    }
    delay(50);
  }
  motors_state = motors_state_t::WAITING;
}

float droite (float x, float y ){
  float z = sqrt(pow(x,2) + pow(y,2));
  return z;
}

// parametre en millimetre , x et y coordonnées que l'ont veut atteindre,
// theta parametre d'orientation en radian
void aller_a_position(float x, float y, float theta) {
  float dx = x - odometry.current.x*1000;
  float dy = y - odometry.current.y*1000;
  if ((abs(dx) < 5 && abs(dy) < 5 ) && abs(theta - odometry.current.theta) < 0.001) {
    Serial.println("Pas de mouvements nécessaire");
    return;
  }
  Serial.printf("Moving by x: %.2f, y:%.2f\n", dx, dy);
  float move_theta = atan2(dy, dx);
  float d_move_theta = move_theta - odometry.current.theta;
  float d = droite(dx, dy);
  if (abs(dx) > 5 || abs(dy) > 5 ) {
    tourner(d_move_theta);
    avancer(d);
  }
  tourner(-clamp_angle(odometry.current.theta - theta));

  Serial.printf("dx: %.2f, dy: %.2f, dθ: %.2f\n", dx, dy, d_move_theta);
  motors_state = motors_state_t::WAITING;
}
//- rajouter une procédure en cas d'obstacle pour le contourner ou simplement créer une nouvelle trajectoire non linéaire

void MoveTask(void *pvParams){
  Serial.println("Started move task");
  while (1) {
    if (motors_state == motors_state_t::WAITING) {
      float dx = target_pos.x*1000 - odometry.current.x*1000;
      float dy = target_pos.y*1000 - odometry.current.y*1000;
      if (!((abs(dx) < 5 && abs(dy) < 5 ) && abs(target_pos.theta - odometry.current.theta) < 0.001)) {
        aller_a_position(target_pos.x*1000, target_pos.y*1000, target_pos.theta);
      }
    }
    delay(50);
  }
}

void doGoPos(char *cmd) {
  aller_a_position(target_pos.x*1000, target_pos.y*1000, target_pos.theta);
  Serial.printf("going to %.2f;%.2f\n", target_pos.x, target_pos.y);
}

// Position is stored in m, but displayed and inputed in cm
void doSetTargetX(char *cmd) {
  float target_x;
  command.scalar(&target_x, cmd);
  target_pos.x = target_x/100;
  Serial.printf("Set target X to %.1fcm\n", target_x);
}

void doSetTargetT(char *cmd) {
  float target_theta;
  command.scalar(&target_theta, cmd);
  target_pos.theta = target_theta*PI/180;
  Serial.printf("Set target θ to %.1f°\n", target_theta);
}

void doSetTargetY(char *cmd) {
  float target_y;
  command.scalar(&target_y, cmd);
  target_pos.y = target_y/100;
  Serial.printf("Set target Y to %.1fcm\n", target_y);
}

void doSetCurrentX(char *cmd) {
  float current_x;
  command.scalar(&current_x, cmd);
  odometry.current.x = current_x/100;
  Serial.printf("Set current X to %.1fcm\n", current_x);
}

void doSetCurrentY(char *cmd) {
  float current_y;
  command.scalar(&current_y, cmd);
  odometry.current.y = current_y/100;
  Serial.printf("Set current Y to %.5fcm\n", current_y);
}

void doPrintOdoStatus(char *cmd) {
  Serial.printf("Current X:\t %.1fcm\n", odometry.current.x*100);
  Serial.printf("Current Y:\t %.1fcm\n", odometry.current.y*100);
  Serial.printf("Current θ:\t %.1f°\n", odometry.current.theta*180/PI);

  Serial.printf("Target X:\t %.1fcm\n", target_pos.x*100);
  Serial.printf("Target Y:\t %.1fcm\n", target_pos.y*100);
  Serial.printf("Target θ:\t %.1f°\n", target_pos.theta*180/PI);
}

void setup() {
  Serial.begin(115200);
  // Serial.printf("Connecting to ap: %s\n", ENV_WIFI_SSID);
  // IPAddress agent_ip(ENV_AGENT_IP);
  // uint16_t agent_port = 8888;
  // set_microros_wifi_transports(ENV_WIFI_SSID, ENV_WIFI_PASSWORD, agent_ip, agent_port);
  // delay(2000);
  // allocator = rcl_get_default_allocator();
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // RCCHECK(rclc_node_init_default(&node, "micro_ros_wifi_node", "", &support));
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  //   "obstacle"));
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &publisher_pose,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
  //   "pami/position"));
  // delay(2000);

  Wire.setPins(14, 13);
  lox.begin(); // TODO: check for init
  int v = 1;
  int f = v*51200;

  FastLED.addLeds<WS2812B, RGB_PIN, GRB>(led, 1);

  xTaskCreatePinnedToCore(StopTask, "StopTask", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(MoveTask, "MoveTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(LightTask, "LightTask", 2048, NULL, 2, NULL, 1);
  // xTaskCreatePinnedToCore(RosTask, "RosTask", 4096, NULL, 2, NULL, 1);

  // Bras servo init
  if (ledcAttach(SERVO_PIN, 60, 12))
    Serial.println("init");

  engine.init();
  stepper1.init();
  stepper2.init();
  motors_state = motors_state_t::WAITING;
  xTaskCreatePinnedToCore(OdoTask, "OdoTask", 2048, NULL, 2, NULL, 1);

  // Commander
  command.verbose = VerboseMode::user_friendly;
  command.decimal_places = 5;
  command.add('G', doGoPos);
  command.add('X', doSetTargetX);
  command.add('Y', doSetTargetY);
  command.add('T', doSetTargetT);
  command.add('i', doSetCurrentX);
  command.add('j', doSetCurrentY);
  command.add('P', doPrintOdoStatus);
}

void loop() {
  command.run();
  delay(10);
}