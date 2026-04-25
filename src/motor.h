#pragma once
#include <FastAccelStepper.h>


class Stepper : public FastAccelStepper {
public:
    Stepper(uint8_t dir_pin, uint8_t step_pin, int8_t sleep_pin, uint8_t vpwm_pin, uint8_t en_pin, uint32_t current);
    void init();
    MoveResultCode move(int32_t move, bool blocking = false);
    bool isRunning() const;
    void stopMove();
     
private:
    FastAccelStepper* m_stepper;
    uint8_t m_step_pin;
    uint8_t m_dir_pin;
    uint8_t m_sleep_pin;
    uint8_t m_vpwm_pin;
    uint8_t m_en_pin;
    uint32_t m_current;
};