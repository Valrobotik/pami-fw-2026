#pragma once
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMC2209.h>

#include "pin_definitions.h"


typedef enum motors_state_t {
  OFF,
  TURNING,
  FORWARD,
  WAITING,
  STOPPING_TURNING,
  STOPPING_FORWARD,
} motors_state_t;

extern motors_state_t motors_state;

class Stepper {
public:
#if ENV_REV == 1
    Stepper(uint8_t dir_pin, uint8_t step_pin, int8_t sleep_pin, uint8_t vpwm_pin, uint8_t en_pin, uint32_t current);
#elif ENV_REV == 2
    Stepper(uint8_t dir_pin, uint8_t step_pin, uint8_t en_pin, uint32_t current, TMC2209::SerialAddress addr);
#endif
    void init();
    MoveResultCode move(int32_t move, bool blocking = false);
    bool isRunning() const;
    bool isStopping() const;
    bool isMoving() const;
    void stopMove();
    int32_t getCurrentPosition() const;
    bool disableOutputs();
    void disable();
    void enable();
     
private:
    FastAccelStepper* m_stepper;
#if ENV_REV == 1
    uint8_t m_step_pin;
    uint8_t m_dir_pin;
    uint8_t m_sleep_pin;
    uint8_t m_vpwm_pin;
    uint8_t m_en_pin;
    uint32_t m_current;
#elif ENV_REV == 2
    TMC2209::SerialAddress m_addr;
    TMC2209 m_driver;
    uint8_t m_step_pin;
    uint8_t m_dir_pin;
    uint8_t m_en_pin;
    uint32_t m_current;
#endif
};