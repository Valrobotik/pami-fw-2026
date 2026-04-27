#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include "Commander.h"
#include <ESP32Servo.h>

#include "pin_definitions.h"
#include "motor.h"

#define DIAMETRE_ROUE 65// en millimètre
#define LARGEUR 123// en millimètre largeur entre les 2 roues
#define DISTANCE 100 // en millimetre, distance avant le mur
#define LONGUEURCAPLA 100 // en millimetre, longueur d'un obstacle

Servo bras;


Adafruit_VL53L0X lox = Adafruit_VL53L0X();
 
Commander command = Commander(Serial);

extern FastAccelStepperEngine engine;
Stepper stepper1 = Stepper(DIR_1_PIN, STEP_1_PIN, SLEEP_1_PIN, VREF_1_PIN, EN_1_PIN, 300);
Stepper stepper2 = Stepper(DIR_2_PIN, STEP_2_PIN, SLEEP_2_PIN, VREF_2_PIN, EN_2_PIN, 300);

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
  STOPPING,
} motors_state_t;
motors_state_t motors_state = motors_state_t::OFF;

QueueHandle_t stop_queue;
void StopTask(void *pvParams) {
  Serial.println("Started stop task");
  while (1) {
    if (motors_state == motors_state_t::FORWARD ||
        motors_state == motors_state_t::TURNING) {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      if (measure.RangeMilliMeter < DISTANCE) {
        xQueueSend(stop_queue, NULL, 0); // TODO: change
      }
    }
    delay(50);
  }
}

void bras_noisette(){
  bras.setPeriodHertz(50); // Fréquence PWM pour le bras
  bras.attach(39); // Largeur minimale et maximale de l'impulsion (en µs) pour aller de 0° à 180°
  for (int pos = 0; pos <= 180; pos += 1) {  // go from 0-180 degrees
    bras.write(pos);}
  for (int pos = 180; pos >= 0 ; pos += 1) {  // go from 180-0 degrees
    bras.write(pos);}
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

int avancer(int d){ // parametre en millimetre
  motors_state = motors_state_t::FORWARD;
  int32_t start_steps = stepper1.getCurrentPosition();
  stepper2.move(distance_to_step(d));
  stepper1.move(-distance_to_step(d));
  
  while (stepper2.isRunning()&& stepper1.isRunning()){
    if (xQueueReceive(stop_queue, NULL, pdMS_TO_TICKS(50)) == pdTRUE) {
      stepper2.stopMove();
      stepper1.stopMove();
      delay(1000); // prsk c'est comme ca
      float distance_parcourue = step_to_distance(abs(start_steps-stepper1.getCurrentPosition()));
      motors_state = motors_state_t::WAITING;
      return distance_parcourue;
    }
  }
  motors_state = motors_state_t::WAITING;
  return d;
}

int tourner(int rot){ // paramètre en degré, largeur en millimetre 
  motors_state = motors_state_t::TURNING;
  int32_t start_steps = stepper2.getCurrentPosition();
  stepper2.move(distance_to_step((LARGEUR*PI*rot)/360));
  stepper1.move(-distance_to_step((-LARGEUR*PI*rot)/360));
  while (stepper2.isRunning()&& stepper1.isRunning()){
    if (xQueueReceive(stop_queue, NULL, pdMS_TO_TICKS(50)) == pdTRUE) {
      stepper2.stopMove();
      stepper1.stopMove();
      motors_state = motors_state_t::STOPPING;
      delay(1000); // prsk c'est comme ca
      float distance_parcourue = -step_to_distance(abs(start_steps-stepper2.getCurrentPosition()));
      // Serial.printf("distance parcourue: %f\n", distance_parcourue);
      // Serial.printf("returned: %d\n", int((distance_parcourue * 360)/(LARGEUR*PI)));
      motors_state = motors_state_t::WAITING;
      return int((distance_parcourue * 360)/(LARGEUR*PI));
    }
  }
  motors_state = motors_state_t::WAITING;
  return rot;
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

  odometry_status.current_angle = (odometry_status.current_angle + tourner(d_theta)) % 360;
  int distance_parcourue = avancer(z);
  if (distance_parcourue == z) {
    odometry_status.current_x += dx;
    odometry_status.current_y += dy;
  } else {
    odometry_status.current_x += distance_parcourue * cos(theta);
    odometry_status.current_y+= distance_parcourue * sin(theta);
  }

  Serial.printf("dx: %d, dy: %d, dθ: %d\n", dx, dy, d_theta);
  Serial.printf("moving by %d from %d to %d\n", d_theta, bf, odometry_status.current_angle);
}
// A faire : -rajouter une position initiale de réference !
//- rajouter une procédure en cas d'obstacle pour le contourner ou simplement créer une nouvelle trajectoire non linéaire
// (a ce propos là refléchir à une trajectoire circulaire en utilisant séparenemen les 2 roues et pas le déplacement central -> permettrait une trajectoire plus souple)
// emplacement initial avec un pole pour reperer son orientation et des coordonnées 0,0 => à établier avec le terrain de jeu directement
// Coordonnées des capla à avoir avec le wifi, sinon pas possible de différencier le mur et les capla et donc ce qu'il faudrait contourner dans la trajectoire
// Donc faire une fonction capla qui connait la coordonées du capla à eviter si il est sur notre droit de trajectoire et decaler notre trajectoire

void MoveTask(void *pvParams){
  Serial.println("Started move task");
  while (1) {
    if (motors_state == motors_state_t::WAITING &&
       (odometry_status.current_x != odometry_status.target_x ||
        odometry_status.current_x != odometry_status.target_x)) {
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
  Wire.setPins(14, 13);
  lox.begin(); // TODO: check for init
  int v = 1;
  int f = v*51200;

  stop_queue = xQueueCreate(1, 0);
  xTaskCreate(StopTask, "StopTask", 2048, NULL, 2, NULL);
  xTaskCreate(MoveTask, "MoveTask", 2048, NULL, 2, NULL);

  /*Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);*/

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


  
  bras_noisette();
}

void loop() {
  command.run(); 
  delay(10);
}