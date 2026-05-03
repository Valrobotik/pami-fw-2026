#include "kinematics.h"

int distance_to_step(float d) { //d en millimètre distance à parcourir
  return (d*STEPS_PER_ROT)/(DIAMETRE_ROUE*PI);
}

float step_to_distance(int s) {
  return (s*DIAMETRE_ROUE*PI)/STEPS_PER_ROT;
}

void avancer(float d, bool override){ // parametre en millimetre
  motors_state = motors_state_t::FORWARD;
  stepper1.move(-distance_to_step(d));
  stepper2.move(distance_to_step(d));
  while ((stepper2.isMoving() && stepper1.isMoving())){
    if ((obstacle_detected || new_pose) && !override) {
      new_pose = false;
      stepper2.stopMove();
      stepper1.stopMove();
      motors_state = motors_state_t::STOPPING_FORWARD;
    }
    delay(50);
  }
  motors_state = motors_state_t::WAITING;
}

void tourner(float rot, bool override){ // rot en radian [-pi;pi]
  motors_state = motors_state_t::TURNING;
  int32_t steps = -distance_to_step(LARGEUR*rot/2);
  stepper2.move(steps);
  stepper1.move(steps);
  while ((stepper2.isMoving() && stepper1.isMoving())){
    if ((obstacle_detected || new_pose) && !override) {
      new_pose = false;
      stepper2.stopMove();
      stepper1.stopMove();
      motors_state = motors_state_t::STOPPING_TURNING;
    }
    delay(50);
  }
  motors_state = motors_state_t::WAITING;
}

float droite(float x, float y ) {
  float z = sqrt(pow(x,2) + pow(y,2));
  return z;
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

bool target_reached() {
  float dx = target_pos.x*1000 - odometry.current.x*1000;
  float dy = target_pos.y*1000 - odometry.current.y*1000;
  return (abs(dx) < 5 && abs(dy) < 5 ) && abs(target_pos.theta - odometry.current.theta) < 0.001;
}

// parametre en millimetre , x et y coordonnées que l'ont veut atteindre,
// theta parametre d'orientation en radian
void aller_a_position(float x, float y, float theta) {
  float dx = x - odometry.current.x*1000;
  float dy = y - odometry.current.y*1000;
  if (target_reached()) {
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

void OdoTask(void *pvParams) {
  while (1) {
    int32_t d_step_1 = -(stepper1.getCurrentPosition() - odometry.last_stepper_1);
    odometry.last_stepper_1 = stepper1.getCurrentPosition();
    int32_t d_step_2 = stepper2.getCurrentPosition() - odometry.last_stepper_2;
    odometry.last_stepper_2 = stepper2.getCurrentPosition();
    float distance_moyenne = (step_to_distance(d_step_1) + step_to_distance(d_step_2))/2;
    float d_theta = (step_to_distance(d_step_1) - step_to_distance(d_step_2))/LARGEUR;
    float new_theta = odometry.current.theta + d_theta;
    odometry.current.theta = new_theta;
    odometry.current.theta = clamp_angle(odometry.current.theta);
    odometry.current.x += distance_moyenne*cos(new_theta)/1000;
    odometry.current.y += distance_moyenne*sin(new_theta)/1000;
    delay(10);
  }
}

void MoveTask(void *pvParams){
  Serial.println("Started move task");
  while (1) {
    if (motors_state == motors_state_t::WAITING && !obstacle_detected) {
      if (!target_reached()) {
        aller_a_position(target_pos.x*1000, target_pos.y*1000, target_pos.theta);
      }
    }
    delay(50);
  }
}
