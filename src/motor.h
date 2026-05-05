#pragma once
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMC2209.h>


typedef enum motors_state_t {
  OFF,
  TURNING,
  FORWARD,
  WAITING,
  STOPPING_TURNING,
  STOPPING_FORWARD,
} motors_state_t;

class Stepper {
public:
    Stepper(uint8_t dir_pin, uint8_t step_pin, uint8_t en_pin, uint32_t current);
    void init();
    MoveResultCode move(int32_t move, bool blocking = false);
    bool isRunning() const;
    bool isStopping() const;
    bool isMoving() const;
    void stopMove();
    int32_t getCurrentPosition() const;
    bool disableOutputs();
     
private:
    FastAccelStepper* m_stepper;
    TMC2209 m_driver;
    uint8_t m_step_pin;
    uint8_t m_dir_pin;
    uint8_t m_en_pin;
    uint32_t m_current;
};