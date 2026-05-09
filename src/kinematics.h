#pragma once
#include "motor.h"

#define DIAMETRE_ROUE 66// en millimètre
#define LARGEUR 123// en millimètre largeur entre les 2 roues
#define STEPS_PER_ROT 51200

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

extern motors_state_t motors_state;
extern pose_2d_t target_pos;
extern Stepper stepper1;
extern Stepper stepper2;
extern odometry_status_t odometry;

extern bool obstacle_detected;
extern bool new_pose;

int distance_to_step(float d); //d en millimètre distance à parcourir
float step_to_distance(int s);
void avancer(float d, bool override = false); // parametre en millimetre
void tourner(float rot, bool override = false); // rot en radian [-pi;pi]
float droite(float x, float y);
float clamp_angle(float theta);
void aller_a_position(float x, float y, float theta);
void OdoTask(void *pvParams);
void MoveTask(void *pvParams);
bool target_reached();