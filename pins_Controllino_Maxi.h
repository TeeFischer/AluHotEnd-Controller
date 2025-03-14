/**
  * Pin definitions on an Controllino Maxi Automation
  */

#include <Controllino.h>

// thermoelement modules Pin
#define thermoSO  CONTROLLINO_PIN_HEADER_MISO
#define thermoSCK CONTROLLINO_PIN_HEADER_SCK
#define thermoCS1 CONTROLLINO_PIN_HEADER_SS // CONTROLLINO_D1
#define thermoCS2 CONTROLLINO_D3
#define thermoCS3 CONTROLLINO_D5

// PWM Pin (heater)
#define pwmPin CONTROLLINO_D6 // 230v SSR
//#define pwmPin CONTROLLINO_D7 // 24v MOSFET

// eStop Pin
#define eStopPin CONTROLLINO_DI0

// motor Pins
#define dirPin CONTROLLINO_D2
#define stepPin CONTROLLINO_D0
#define endstopPin 99
