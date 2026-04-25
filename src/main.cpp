#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include "Commander.h"

#include "pin_definitions.h"

#define DIAMETRE_ROUE 65// en millimètre
#define LARGEUR 123// en millimètre largeur entre les 2 roues
#define DISTANCE 100 // en millimetre, distance avant le mur
#define LONGUEURCAPLA 100 // en millimetre, longueur d'un obstacle

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
 
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

Commander command = Commander(Serial);

typedef struct odometry_status_t {
  int current_x = 0;
  int current_y = 0;
  int current_angle = 0;

  int target_x = 0;
  int target_y = 0;
  int target_angle = 0;
} odometry_status_t;
odometry_status_t odometry_status {};

void init_moteur1(){
  int v = 1;
  int f = v*51200 ;
  pinMode(SLEEP_1_PIN, OUTPUT);
  pinMode(VREF_1_PIN, OUTPUT);
  pinMode(EN_1_PIN,OUTPUT);
  digitalWrite(EN_1_PIN, HIGH);
  digitalWrite(SLEEP_1_PIN, HIGH);
  ledcAttach(VREF_1_PIN, 5000, 12);
  ledcWrite(VREF_1_PIN, 300);
  stepper1 = engine.stepperConnectToPin(STEP_1_PIN);
  if (stepper1) {
    stepper1->setDirectionPin(DIR_1_PIN);
    //stepper->setEnablePin(EN_1);
    stepper1->setAutoEnable(true);
  }
  stepper1->setSpeedInHz(f);       // 500 steps/s
  stepper1->setAcceleration(100000); 
}

void init_moteur2(){
  int v = 1;
  int f = v*51200 ;
  pinMode(SLEEP_2_PIN, OUTPUT);
  pinMode(VREF_2_PIN, OUTPUT);
  pinMode(EN_2_PIN,OUTPUT);
  digitalWrite(EN_2_PIN, HIGH);
  digitalWrite(SLEEP_2_PIN, HIGH);
  ledcAttach(VREF_2_PIN, 5000, 12);
  ledcWrite(VREF_2_PIN, 300);
  stepper2 = engine.stepperConnectToPin(STEP_2_PIN);
  if (stepper2) {
    stepper2->setDirectionPin(DIR_2_PIN);
    stepper2->setAutoEnable(true);
    
  }
  stepper2->setSpeedInHz(f);       // 500 steps/s
  stepper2->setAcceleration(100000);  // 100 steps/s²
}

bool obstacle(){ /*Est ce que mur présent devant la tete du robot*/
  return false; // tmp
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  delay(10);    
  // Serial.println(measure.RangeMilliMeter);
  return (measure.RangeMilliMeter < DISTANCE);
}

int distance (int d ){ //d en millimètre distance à parcourir
  float var;
  int pas;
  var = (d)/ (DIAMETRE_ROUE*PI);
  pas = var * 51200 ;
  return pas;
}

void avancer(int d){ // parametre en millimetre
  stepper2->move(distance(d));
  stepper1->move(-distance(d));
  
  while (stepper2->isRunning()&& stepper1->isRunning()){
    if (obstacle() == true){
      // Serial.printf("%d la fonction avance avec ces paramètres\n" , d);
      stepper2->stopMove();
      stepper1->stopMove();
    }
  }
}

void tourner(int rot){ // paramètre en degré, largeur en millimetre 
  stepper2->move(distance((LARGEUR*PI*rot)/360));
  stepper1->move(-distance((-LARGEUR*PI*rot)/360));
  // Serial.printf("%d %d la fonction tourne avec ces paramètres\n", rot);
  while (stepper2->isRunning()&& stepper1->isRunning()){
    if (obstacle() == true){
      stepper2->stopMove();
      stepper1->stopMove();
    }
  }
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
  Serial.printf("theta: %d\n", theta);
  int d_theta = ((theta - odometry_status.current_angle + 180) % 360) - 180;
  int z = droite(dx, dy);
  tourner(d_theta);
  avancer(z);
  // odometry_status.current_angle += d_theta;
  int bf = odometry_status.current_angle;
  odometry_status.current_angle = (odometry_status.current_angle + d_theta) % 360;
  odometry_status.current_x += dx;
  odometry_status.current_y += dy;
  Serial.printf("dx: %d, dy: %d, dθ: %d\n", dx, dy, d_theta);
  Serial.printf("moving by %d from %d to %d\n", d_theta, bf, odometry_status.current_angle);
  // if (obstacle() == true){
  //   tourner(90);
  //   avancer(LONGUEURCAPLA);
  //   tourner(-90);
  //   avancer(droite(x,y) - droite(current_x, current_y));
  // }
  
}
// A faire : -rajouter une position initiale de réference !
//- rajouter une procédure en cas d'obstacle pour le contourner ou simplement créer une nouvelle trajectoire non linéaire
// (a ce propos là refléchir à une trajectoire circulaire en utilisant séparenemen les 2 roues et pas le déplacement central -> permettrait une trajectoire plus souple)
// emplacement initial avec un pole pour reperer son orientation et des coordonnées 0,0 => à établier avec le terrain de jeu directement
// Coordonnées des capla à avoir avec le wifi, sinon pas possible de différencier le mur et les capla et donc ce qu'il faudrait contourner dans la trajectoire
// Donc faire une fonction capla qui connait la coordonées du capla à eviter si il est sur notre droit de trajectoire et decaler notre trajectoire

void doGoPos(char *cmd) {
  aller_a_position(odometry_status.target_x, odometry_status.target_y);
  Serial.println("going to 100;100");
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
  lox.begin();
  int v = 1;
  int f = v*51200 ;

  /*Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);*/

  Serial.println(f);
  engine.init();
  init_moteur2();
  init_moteur1();

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

uint32_t elapsed = 0;
bool a = false;

void loop() {
  //  if (millis() > elapsed + 2000) {
  //    a = !a;
  //    elapsed = millis();
  //  }
  command.run(); 
}