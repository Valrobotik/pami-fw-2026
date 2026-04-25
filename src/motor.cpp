#include "motor.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();

Stepper::Stepper(uint8_t dir_pin, uint8_t step_pin, int8_t sleep_pin, uint8_t vpwm_pin, uint8_t en_pin, uint32_t current)
    : m_stepper(NULL), m_dir_pin(dir_pin), m_step_pin(step_pin), m_sleep_pin(sleep_pin),
      m_vpwm_pin(vpwm_pin), m_en_pin(en_pin), m_current(current) {}

void Stepper::init() {
  int v = 1;
  int f = v*51200 ;
  pinMode(m_sleep_pin, OUTPUT);
  pinMode(m_vpwm_pin, OUTPUT);
  pinMode(m_en_pin,OUTPUT);
  ledcAttach(m_vpwm_pin, 5000, 12);

  digitalWrite(m_en_pin, HIGH);
  digitalWrite(m_sleep_pin, HIGH);
  ledcWrite(m_vpwm_pin, 300);
  m_stepper = engine.stepperConnectToPin(m_step_pin);
  if (m_stepper) {
    m_stepper->setDirectionPin(m_dir_pin);
    //stepper->setEnablePin(EN_1);
    m_stepper->setAutoEnable(true);
  }
  m_stepper->setSpeedInHz(f);       // 500 steps/s
  m_stepper->setAcceleration(100000); 
}

MoveResultCode Stepper::move(int32_t move, bool blocking)
{
    return m_stepper->move(move, blocking);
}

bool Stepper::isRunning() const
{
    return m_stepper->isRunning();
}

void Stepper::stopMove() {
    return m_stepper->stopMove();
}
