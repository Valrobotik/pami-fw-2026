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

typedef struct odometry_status_t {
  int current_x = 0;
  int current_y = 0;
  int current_angle = 0;

  int target_x = 0;
  int target_y = 0;
  int target_angle = 0;
} odometry_status_t;
odometry_status_t odometry_status {};

typedef enum motors_state_t {
  OFF,
  TURNING,
  FORWARD,
  WAITING,
  STOPPING_TURNING,
  STOPPING_FORWARD,
} motors_state_t;
motors_state_t motors_state = motors_state_t::OFF;

QueueHandle_t stop_queue;
bool obstacle_detected = false;
void StopTask(void *pvParams) {
  Serial.println("Started stop task");
  while (1) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeMilliMeter < DISTANCE) {
      obstacle_detected = true;
      xQueueSend(stop_queue, NULL, 0); // TODO: change
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

void ros_update_odometry() {
  msg_pose.header.stamp.sec = 0;
  msg_pose.header.stamp.nanosec = 0;
  rosidl_runtime_c__String__assign(&msg_pose.header.frame_id, "world");
  msg_pose.pose.position.x = ((double) odometry_status.current_x)/1000;
  msg_pose.pose.position.y = ((double)odometry_status.current_y)/1000;
  msg_pose.pose.position.z = 0;
  msg_pose.pose.orientation.x = 0;
  msg_pose.pose.orientation.y = 0;
  double half_theta = (odometry_status.current_angle * PI / 180.0) * 0.5;
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

int distance_to_step(int d){ //d en millimètre distance à parcourir
  float var;
  int pas;
  var = (d)/ (DIAMETRE_ROUE*PI);
  pas = var * 51200 ;
  return pas;
}

float step_to_distance(int s) {
  return (s*DIAMETRE_ROUE*PI)/51200;
}

void avancer(int d){ // parametre en millimetre
  motors_state = motors_state_t::FORWARD;
  int32_t start_steps = stepper2.getCurrentPosition();
  while (stepper2.isRunning()&& stepper1.isRunning()){
    if (xQueueReceive(stop_queue, NULL, pdMS_TO_TICKS(50)) == pdTRUE) {
      stepper2.stopMove();
      stepper1.stopMove();
      motors_state = motors_state_t::STOPPING_FORWARD;
      delay(1000); // prsk c'est comme ca
    }
  }
  motors_state = motors_state_t::WAITING;
}

void tourner(int rot){ // paramètre en degré, largeur en millimetre 
  motors_state = motors_state_t::TURNING;
  stepper2.move(distance_to_step((LARGEUR*PI*rot)/360));
  stepper1.move(-distance_to_step((-LARGEUR*PI*rot)/360));
  while (stepper2.isRunning()&& stepper1.isRunning()){
    if (xQueueReceive(stop_queue, NULL, pdMS_TO_TICKS(50)) == pdTRUE) {
      stepper2.stopMove();
      stepper1.stopMove();
      motors_state = motors_state_t::STOPPING_TURNING;
      delay(1000); // prsk c'est comme ca
    }
  }
  motors_state = motors_state_t::WAITING;
}

int droite (int x, int y ){
  int z = sqrt(pow(x,2) + pow(y,2));
  return z;
}

// parametre en millimetre , x et y coordonnées que l'ont veut atteindre,
// theta parametre d'orientation
void aller_a_position(int x ,int y) {
  int dx = x - odometry_status.current_x;
  int dy = y - odometry_status.current_y;
  if (!dx && !dy) {
    Serial.println("Pas de mouvements nécessaire");
    return;
  }
  Serial.printf("Moving by x: %d, y:%d\n", dx, dy);
  int theta = (int(360 - atan2(dy, dx)*(180/PI))) % 360;
  int d_theta = ((theta - odometry_status.current_angle + 180) % 360) - 180;
  int z = droite(dx, dy);
  int bf = odometry_status.current_angle;
  tourner(d_theta);
  avancer(z);

  Serial.printf("dx: %d, dy: %d, dθ: %d\n", dx, dy, d_theta);
  Serial.printf("moving by %d from %d to %d\n", d_theta, bf, odometry_status.current_angle);
  motors_state = motors_state_t::WAITING;
}
//- rajouter une procédure en cas d'obstacle pour le contourner ou simplement créer une nouvelle trajectoire non linéaire

void MoveTask(void *pvParams){
  Serial.println("Started move task");
  while (1) {
    if (motors_state == motors_state_t::WAITING &&
       (odometry_status.current_x != odometry_status.target_x ||
        odometry_status.current_y != odometry_status.target_y)) {
          Serial.println("Target not reached, moving again....");
          aller_a_position(odometry_status.target_x, odometry_status.target_y);
        }
    delay(50);
  }
}

void doGoPos(char *cmd) {
  aller_a_position(odometry_status.target_x, odometry_status.target_y);
  Serial.printf("going to %d;%d\n", odometry_status.target_x, odometry_status.target_y);
}

void doSetTargetX(char *cmd) {
  float target_x;
  command.scalar(&target_x, cmd);
  odometry_status.target_x = int(round(target_x));
  Serial.printf("Set target X to %d\n", odometry_status.target_x);
}

void doSetTargetY(char *cmd) {
  float target_y;
  command.scalar(&target_y, cmd);
  odometry_status.target_y = int(round(target_y));
  Serial.printf("Set target Y to %d\n", odometry_status.target_y);
}

void doSetCurrentX(char *cmd) {
  float current_x;
  command.scalar(&current_x, cmd);
  odometry_status.current_x = int(round(current_x));
  Serial.printf("Set current X to %d\n", odometry_status.current_x);
}

void doSetCurrentY(char *cmd) {
  float current_y;
  command.scalar(&current_y, cmd);
  odometry_status.current_x = int(round(current_y));
  Serial.printf("Set current X to %d\n", odometry_status.current_y);
}

void doPrintOdoStatus(char *cmd) {
  Serial.printf("Current X:\t %d\n", odometry_status.current_x);
  Serial.printf("Current Y:\t %d\n", odometry_status.current_y);
  Serial.printf("Current θ:\t %d\n", odometry_status.current_angle);

  Serial.printf("Target X:\t %d\n", odometry_status.target_x);
  Serial.printf("Target Y:\t %d\n", odometry_status.target_y);
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Connecting to ap: %s\n", ENV_WIFI_SSID);
  IPAddress agent_ip(ENV_AGENT_IP);
  uint16_t agent_port = 8888;
  set_microros_wifi_transports(ENV_WIFI_SSID, ENV_WIFI_PASSWORD, agent_ip, agent_port);
  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_wifi_node", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "obstacle"));
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_pose,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
    "pami/position"));
  delay(2000);

  Wire.setPins(14, 13);
  lox.begin(); // TODO: check for init
  int v = 1;
  int f = v*51200;

  stop_queue = xQueueCreate(1, 0);
  xTaskCreate(StopTask, "StopTask", 2048, NULL, 2, NULL);
  xTaskCreate(MoveTask, "MoveTask", 2048, NULL, 2, NULL);
  xTaskCreate(LightTask, "LightTask", 2048, NULL, 2, NULL);
  xTaskCreate(RosTask, "RosTask", 4096, NULL, 2, NULL);
  
  FastLED.addLeds<WS2812B, RGB_PIN, GRB>(led, 1);

  // Bras servo init
  if (ledcAttach(SERVO_PIN, 60, 12))
    Serial.println("init");

  engine.init();
  stepper1.init();
  stepper2.init();
  motors_state = motors_state_t::WAITING;

  // Commander
  command.verbose = VerboseMode::user_friendly;
  command.decimal_places = 5;
  command.add('G', doGoPos);
  command.add('X', doSetTargetX);
  command.add('Y', doSetTargetY);
  command.add('i', doSetCurrentX);
  command.add('j', doSetCurrentY);
  command.add('P', doPrintOdoStatus);
}

void loop() {
  command.run();
  delay(10);
}