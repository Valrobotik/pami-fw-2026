#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include "Commander.h"
#include <FastLED.h>


#include "pin_definitions.h"
#include "motor.h"
#include "kinematics.h"
#include "ros.h"

#define DISTANCE 100 // en millimetre, distance avant le mur

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
 
Commander command = Commander(Serial);

extern FastAccelStepperEngine engine;
Stepper stepper1 = Stepper(DIR_1_PIN, STEP_1_PIN, SLEEP_1_PIN, VREF_1_PIN, EN_1_PIN, 300);
Stepper stepper2 = Stepper(DIR_2_PIN, STEP_2_PIN, SLEEP_2_PIN, VREF_2_PIN, EN_2_PIN, 300);

CRGB led[1];

odometry_status_t odometry = {0};
pose_2d_t target_pos = {0};

motors_state_t motors_state = motors_state_t::OFF;

extern states state;

bool noisette = false;

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

void NoisetteTask(void *pvParams) {
  Serial.println("Started noisette task");
  while (1) {
    if (noisette) {
      for (int pos = 0; pos < 256; pos++) {  // go from 0-180 degrees
        ledcWrite(SERVO_PIN, pos);
        delay(10);
      }
      for (int pos = 255; pos >= 0; pos--) {  // go from 180-0 degrees
        ledcWrite(SERVO_PIN, pos);
        delay(10);
      }
    }
    delay(100);
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
    if (state != states::AGENT_CONNECTED && (millis() % 500) < 200 && (millis() % 500) > 100) {
      led[0] = CRGB::Purple;
    }
    FastLED.show();
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
  xTaskCreatePinnedToCore(LightTask, "LightTask", 2048, NULL, 2, NULL, 1);
  init_ros();
  Wire.setPins(14, 13);
  lox.begin(); // TODO: check for init
  int v = 1;
  int f = v*51200;

  FastLED.addLeds<WS2812B, RGB_PIN, GRB>(led, 1);

  xTaskCreatePinnedToCore(StopTask, "StopTask", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(MoveTask, "MoveTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(NoisetteTask, "NoisetteTask", 1024, NULL, 2, NULL, 1);

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
  ros_loop();
  delay(10);
}