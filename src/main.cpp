#include <Arduino.h>
#include "FastAccelStepper.h"
#include "Adafruit_VL53L0X.h"
// #define EN 6
// #define DIR 7
// #define STEP 8
// #define SLEEP 5
// #define VREF 10

//#define EN 18
//#define DIR 33
//#define STEP 48
//#define SLEEP 17
//#define VREF 47

#define EN_1 6
#define DIR_1 7
#define STEP_1 8
#define VREF_1 10
#define SLEEP_1 5
#define FAULT_1 9

#define EN_2 18
#define DIR_2 33
#define STEP_2 48
#define VREF_2 47
#define SLEEP_2 17
#define FAULT_2 38

#define SCL 13
#define SDA 14


#define DIAMETRE_ROUE 65// en millimètre
#define LARGEUR 123// en millimètre largeur entre les 2 roues
#define DISTANCE 100 // en millimetre, distance avant le mur

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
 

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void init_moteur1(){
  int v = 1;
  int f = v*51200 ;
  pinMode(SLEEP_1, OUTPUT);
   pinMode(VREF_1, OUTPUT);
   pinMode(EN_1,OUTPUT);
   digitalWrite(EN_1, HIGH);
   digitalWrite(SLEEP_1, HIGH);
   // analogWrite(VREF, 500);
   ledcAttach(VREF_1, 5000, 12);
   ledcWrite(VREF_1, 512);
   // digitalWrite(VREF, HIGH);
   delay(10);
  stepper1 = engine.stepperConnectToPin(STEP_1);
  if (stepper1) {
    stepper1->setDirectionPin(DIR_1);
    //stepper->setEnablePin(EN_1);
    stepper1->setAutoEnable(true);
  }
  stepper1->setSpeedInHz(f);       // 500 steps/s
  stepper1->setAcceleration(100000); 

}

void init_moteur2(){
  int v = 1;
  int f = v*51200 ;
  pinMode(SLEEP_2, OUTPUT);
  pinMode(VREF_2, OUTPUT);
  pinMode(EN_2,OUTPUT);
  digitalWrite(EN_2, HIGH);
  digitalWrite(SLEEP_2, HIGH);
   // analogWrite(VREF, 500);
   ledcAttach(VREF_2, 5000, 12);
   ledcWrite(VREF_2, 512);
   // digitalWrite(VREF, HIGH);
   delay(10);
  stepper2 = engine.stepperConnectToPin(STEP_2);
  if (stepper2) {
    stepper2->setDirectionPin(DIR_2);
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

void aller_a_position(int x ,int y, int current_x, int current_y, int current_theta){ // parametre en millimetre , x et y coordonnées que l'ont veut atteindre, theta parametre d'orientation
  int theta = atan(y/x)*(180/PI) - current_theta ;
  int z = sqrt(pow(x,2) + pow(y,2));
  tourner(theta);
  avancer(z);
  Serial.printf("%d %d la fonction tourne avec ces paramètres", z, theta);
  current_theta = theta;
  current_x = x;
  current_y=y;
  Serial.printf("%d, %d %d voici les coordonnées apres le déplacement", current_x, current_y, current_theta);
}
// A faire : -rajouter une position initiale de réference !
//- rajouter une procédure en cas d'obstacle pour le contourner ou simplement créer une nouvelle trajectoire non linéaire
// (a ce propos là refléchir à une trajectoire circulaire en utilisant séparenemen les 2 roues et pas le déplacement central -> permettrait une trajectoire plus souple)
// emplacement initial avec un pole pour reperer son orientation et des coordonnées 0,0 => à établier avec le terrain de jeu directement

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