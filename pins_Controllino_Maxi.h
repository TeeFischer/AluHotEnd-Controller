/**
  * Pin definitions on an Controllino Maxi Automation
  */

#include <Controllino.h>

// thermoelement modules Pin
#define thermoSO  CONTROLLINO_PIN_HEADER_MISO // 5
#define thermoSCK CONTROLLINO_PIN_HEADER_SCK // 6
#define thermoCS1 CONTROLLINO_PIN_HEADER_SS // CONTROLLINO_D1  //  2
#define thermoCS2 CONTROLLINO_D3 // 3
#define thermoCS3 CONTROLLINO_D5 // 4

// PWM Pin (heater)
#define pwmPin CONTROLLINO_D6 // 9

// eStop Pin
#define eStopPin CONTROLLINO_A0 // CONTROLLINO_DI0 // 8
