#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Adafruit_VL53L0X.h>

#include "pin_definitions.h"

#define DIAMETRE_ROUE 65// en millimètre
#define LARGEUR 123// en millimètre largeur entre les 2 roues
#define DISTANCE 100 // en millimetre, distance avant le mur
#define LONGUEURCAPLA 100 // en millimetre, longueur d'un obstacle

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
 

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void init_moteur1(){
  int v = 1;
  int f = v*51200 ;
  pinMode(SLEEP_1_PIN, OUTPUT);
   pinMode(VREF_1_PIN, OUTPUT);
   pinMode(EN_1_PIN,OUTPUT);
   digitalWrite(EN_1_PIN, HIGH);
   digitalWrite(SLEEP_1_PIN, HIGH);
   // analogWrite(VREF, 500);
   ledcAttach(VREF_1_PIN, 5000, 12);
   ledcWrite(VREF_1_PIN, 512);
   // digitalWrite(VREF, HIGH);
   delay(10);
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
   // analogWrite(VREF, 500);
   ledcAttach(VREF_2_PIN, 5000, 12);
   ledcWrite(VREF_2_PIN, 512);
   // digitalWrite(VREF, HIGH);
   delay(10);
  stepper2 = engine.stepperConnectToPin(STEP_2_PIN);
  if (stepper2) {
    stepper2->setDirectionPin(DIR_2_PIN);
    stepper2->setAutoEnable(true);
    
  }
  stepper2->setSpeedInHz(f);       // 500 steps/s
  stepper2->setAcceleration(100000);  // 100 steps/s²
}

bool obstacle(){ /*Est ce que mur présent devant la tete du robot*/
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  delay(10);    
  Serial.println(measure.RangeMilliMeter);
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
      Serial.printf("%d la fonction avance avec ces paramètres" , d);
      stepper2->stopMove();
      stepper1->stopMove();
    }
  }
}

void tourner(int rot){ // paramètre en degré, largeur en millimetre 
  stepper2->move(distance((LARGEUR*PI*rot)/360));
  stepper1->move(-distance((-LARGEUR*PI* rot)/360));
  Serial.printf("%d %d la fonction tourne avec ces paramètres", rot);
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


void aller_a_position(int x ,int y, int current_x, int current_y, int current_theta){ // parametre en millimetre , x et y coordonnées que l'ont veut atteindre, theta parametre d'orientation

    int theta = atan(y/x)*(180/PI) - current_theta ;
    int z = droite(x,y);
    tourner(theta);
    avancer(z);
    Serial.printf("%d %d la fonction tourne avec ces paramètres", z, theta);
    current_theta = theta;
    current_x = x;
    current_y=y;
    Serial.printf("%d, %d %d voici les coordonnées apres le déplacement", current_x, current_y, current_theta);
    if (obstacle() == true){
      tourner(90);
      avancer(LONGUEURCAPLA); 
      tourner(-90);
      avancer(droite(x,y) - droite(current_x, current_y));
    }
  
}
// A faire : -rajouter une position initiale de réference !
//- rajouter une procédure en cas d'obstacle pour le contourner ou simplement créer une nouvelle trajectoire non linéaire
// (a ce propos là refléchir à une trajectoire circulaire en utilisant séparenemen les 2 roues et pas le déplacement central -> permettrait une trajectoire plus souple)
// emplacement initial avec un pole pour reperer son orientation et des coordonnées 0,0 => à établier avec le terrain de jeu directement
// Coordonnées des capla à avoir avec le wifi, sinon pas possible de différencier le mur et les capla et donc ce qu'il faudrait contourner dans la trajectoire
// Donc faire une fonction capla qui connait la coordonées du capla à eviter si il est sur notre droit de trajectoire et decaler notre trajectoire


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
  aller_a_position(300 ,300, 0, 0, 0);
}

uint32_t elapsed = 0;
bool a = false;

void loop() {
   if (millis() > elapsed + 2000) {
     a = !a;
     elapsed = millis();
   }
   
  
}