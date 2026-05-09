#pragma once

// HW Rev 01 pin definitions
#if ENV_REV == 1
#define EN_1_PIN 6
#define DIR_1_PIN 7
#define STEP_1_PIN 8
#define VREF_1_PIN 10
#define SLEEP_1_PIN 5
#define FAULT_1 9

#define EN_2_PIN 18
#define DIR_2_PIN 33
#define STEP_2_PIN 48
#define VREF_2_PIN 47
#define SLEEP_2_PIN 17
#define FAULT_2_PIN 38

#define SCL_PIN 13
#define SDA_PIN 14

#define EMS_PIN 4
#define TOF_INT_PIN 12
#define RGB_PIN 21
#define SERVO_PIN 39
#define BUZZER_PIN 16
#define VSENSE_PIN 11

// HW Rev 02 pin definitions
#elif ENV_REV == 2
#define EN_1_PIN 10
#define DIR_1_PIN 8
#define STEP_1_PIN 9

#define EN_2_PIN 33
#define DIR_2_PIN 48
#define STEP_2_PIN 47

#define UART_TX_PIN 6
#define UART_RX_PIN 7

#define SCL_PIN 13
#define SDA_PIN 14

#define EMS_PIN 4
#define TOF_INT_PIN 12
#define RGB_PIN 21
#define SERVO_PIN 39
#define BUZZER_PIN 16
#define VSENSE_PIN 11
#endif