/**
  * PID Autotuning (M303) from https://github.com/MarlinFirmware/Marlin/blob/bugfix-2.1.x/Marlin/src/module/temperature.cpp 
  * as a standalone function
  *
  * Alternately heat and cool the nozzle, observing its behavior to
  * determine the best PID values to achieve a stable temperature.
  * Needs sufficient heater power to make some overshoot at target
  * temperature to succeed.
  */
  
#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <Arduino.h>
#include "max6675.h"

// include a file for pin allocation
//#define _pwmPin 9
#include "pins_Controllino_Maxi.h"

// include extern referenzes
extern void printInfo();  // this function is defined in main.ino
extern MAX6675 thermocouple1; 

// needed macros
#define NOLESS(a, b) if (a < b) a = b;
#define NOMORE(a, b) if (a > b) a = b;
#define ELAPSED(ms, time) ((millis() - (time)) >= (ms))
#define LIMIT(value, lower, upper) if (value < lower) value = lower; else if (value > upper) value = upper;
#define SERIAL_ECHOPGM(...) { \
    char buffer[256]; \
    snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    Serial.print(buffer); \
}
#define SERIAL_ECHOLNPGM(...) { \
    char buffer[256]; \
    snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    Serial.println(buffer); \
}
#define _MIN(a, b) ((a) < (b) ? (a) : (b))
#define MIN_TO_MS(mins) ((mins) * 60000L)

// Text Macros
#define STR_PID_AUTOTUNE "In PID_Autotune: "
#define STR_PID_TEMP_TOO_HIGH "ERROR PID_TEMP_TOO_HIGH"
#define STR_PID_AUTOTUNE_START "PID_AUTOTUNE_START"
#define STR_BIAS "Bias = "
#define STR_D_COLON "; "
#define STR_T_MIN "T_min = "
#define STR_T_MAX "T_max = "
#define STR_KU "K_u = "
#define STR_TU "T_u = "
#define STR_KU "K_u = "
#define STR_CLASSIC_PID "CLASSIC_PID"
#define STR_KP "K_P = "
#define STR_KI "K_I = "
#define STR_KD "K_D = "
#define STR_PID_TIMEOUT "STR_PID_TIMEOUT"
#define STR_PID_AUTOTUNE_FINISHED "STR_PID_AUTOTUNE_FINISHED"

// Funktionsprototypen
void SHV(uint8_t _value);
float degHeater();
void PID_autotune(const float target, const int8_t ncycles, const bool set_result = false);

// Struktur für die PID-Werte
struct raw_pid_t {
  double p;
  double i;
  double d;
};

#endif // AUTOTUNE_H
