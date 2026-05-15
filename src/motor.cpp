#include "motor.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();

#if ENV_REV == 1
Stepper::Stepper(uint8_t dir_pin, uint8_t step_pin, int8_t sleep_pin, uint8_t vpwm_pin, uint8_t en_pin, uint32_t current)
    : m_stepper(NULL), m_dir_pin(dir_pin), m_step_pin(step_pin), m_sleep_pin(sleep_pin),
      m_vpwm_pin(vpwm_pin), m_en_pin(en_pin), m_current(current) {}

#elif ENV_REV == 2
Stepper::Stepper(uint8_t dir_pin, uint8_t step_pin, uint8_t en_pin, uint32_t current, TMC2209::SerialAddress addr)
    : m_stepper(NULL), m_dir_pin(dir_pin), m_step_pin(step_pin), m_en_pin(en_pin), m_current(current), m_addr(addr), m_driver() {}
#endif

void Stepper::init() {
#if ENV_REV == 1
  pinMode(m_sleep_pin, OUTPUT);
  pinMode(m_vpwm_pin, OUTPUT);
  pinMode(m_en_pin,OUTPUT);
  ledcAttach(m_vpwm_pin, 5000, 12);

  digitalWrite(m_en_pin, HIGH);
  digitalWrite(m_sleep_pin, HIGH);
  ledcWrite(m_vpwm_pin, 300);
#elif ENV_REV == 2
  m_driver.setup(Serial2, 19200, m_addr, UART_RX_PIN, UART_TX_PIN);
  m_driver.setHardwareEnablePin(m_en_pin);
  m_driver.useInternalSenseResistors();
  m_driver.enableCoolStep();
//   m_driver.setAllCurrentValues(100, 100, 100);
  m_driver.setRunCurrent(80);
  m_driver.setHoldCurrent(30);
  m_driver.moveUsingStepDirInterface();
  m_driver.setMicrostepsPerStep(256);
  m_driver.enableAutomaticCurrentScaling();
  m_driver.enable();
  Serial2.flush();
  Serial2.end();
#endif
  m_stepper = engine.stepperConnectToPin(m_step_pin);
  if (m_stepper) {
    m_stepper->setDirectionPin(m_dir_pin);
    //stepper->setEnablePin(EN_1);
    m_stepper->setAutoEnable(true);
  }
  m_stepper->setSpeedInHz(51200);     // 500 steps/s
  m_stepper->setAcceleration(100000);
}

MoveResultCode Stepper::move(int32_t move, bool blocking)
{
    if (motors_state == motors_state_t::OFF)
        return MoveResultCode::OK;
    return m_stepper->move(move, blocking);
}

bool Stepper::isRunning() const
{
    return m_stepper->isRunning();
}

bool Stepper::isStopping() const
{
    return m_stepper->isStopping();
}

bool Stepper::isMoving() const
{
    return isRunning() || isStopping();
}

void Stepper::stopMove() {
    return m_stepper->stopMove();
}

int32_t Stepper::getCurrentPosition() const
{
    return m_stepper->getCurrentPosition();
}

bool Stepper::disableOutputs()
{
    return m_stepper->disableOutputs();
}

void Stepper::disable() {
#if ENV_REV == 1
    digitalWrite(m_en_pin, LOW);
#elif ENV_REV == 2
    m_driver.disable();
#endif
}

void Stepper::enable(){
    m_stepper->setCurrentPosition(0);
#if ENV_REV == 1
    digitalWrite(m_en_pin, HIGH);
#elif ENV_REV == 2
    m_driver.enable();
#endif
}
