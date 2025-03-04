/**
  * PID Autotuning (M303) from https://github.com/MarlinFirmware/Marlin/blob/bugfix-2.1.x/Marlin/src/module/temperature.cpp 
  * as a standalone function
  *
  * Alternately heat and cool the nozzle, observing its behavior to
  * determine the best PID values to achieve a stable temperature.
  * Needs sufficient heater power to make some overshoot at target
  * temperature to succeed.
  */

#include "autotune.h"

// settings
#define hotend_max_target 900
#define PID_MAX 255
#define externalWaitForHeatup false
#define MAX_OVERSHOOT_PID_AUTOTUNE 30
#define PID_AUTOTUNE_MAX_CYCLE_MINS 20L

uint8_t T1OutputTune;

// Set Heater Value
// write what ever is needed to set your heater
void SHV(uint8_t _value) {
  T1OutputTune = _value;
  setHeaterValues(T1OutputTune);
}

// returns the HeaterTemp in degrees
// write what ever is needed to set your heater
float degHeater(){
  
  return readTemps();
}

void print_heater_state(){
  printInfo();
}

void _set_heater_pid(raw_pid_t _tune_pid){
  //################################
  // Setze die PID-Werte, z. B. für die Heizung
  double p_value = _tune_pid.p;
  double i_value = _tune_pid.i;
  double d_value = _tune_pid.d;
}

// target in Celsius
void PID_autotune(const float target, const int8_t ncycles, const bool set_result/*=false*/) {
  float current_temp = 0.0;  // in Celsius
  int cycles = 0;
  bool heating = true;

  long next_temp_ms = millis(), t1 = next_temp_ms, t2 = next_temp_ms;
  long t_high = 0, t_low = 0;

  raw_pid_t  tune_pid = { 0, 0, 0 };
  float maxT = 0, minT = 10000;  // in Celsius

  if (target > hotend_max_target) {
    SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TEMP_TOO_HIGH);
    return;
  }

  SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_AUTOTUNE_START);

  SHV(0);

  long bias = PID_MAX >> 1, d = bias;  // bias is set to PID_MAX/2, just in bitwise notation
  SHV(bias);

  // PID Tuning loop

  // with externally initializing the wait_for_heatup value, one can interrupt this loop externally
  #if externalWaitForHeatup
    wait_for_heatup = true;
  #else
    bool wait_for_heatup = true;
  #endif

  while (wait_for_heatup) {

    const long ms = millis();

    // Get the current temperature and constrain it
    current_temp = degHeater();
    NOLESS(maxT, current_temp);
    NOMORE(minT, current_temp);

    if (heating && current_temp > target && ELAPSED(ms, t2 + 5000UL)) {
      heating = false;
      SHV((bias - d) >> 1);
      t1 = ms;
      t_high = t1 - t2;
      maxT = target;
    }

    if (!heating && current_temp < target && ELAPSED(ms, t1 + 5000UL)) {
      heating = true;
      t2 = ms;
      t_low = t2 - t1;
      if (cycles > 0) {
        bias += (d * (t_high - t_low)) / (t_low + t_high);
        LIMIT(bias, 20, PID_MAX - 20);
        d = (bias > PID_MAX >> 1) ? PID_MAX - 1 - bias : bias;

        SERIAL_ECHOPGM(STR_BIAS, bias, STR_D_COLON, d, STR_T_MIN, minT, STR_T_MAX, maxT);
        if (cycles > 2) {
          const float Ku = (4.0f * d) / (3.1415 * (maxT - minT) * 0.5f),    
                      Tu = float(t_low + t_high) * 0.001f,
                      pf = 0.6f,
                      df = 1.0f / 8.0f;

          tune_pid.p = Ku * pf;
          tune_pid.i = tune_pid.p * 2.0f / Tu;
          tune_pid.d = tune_pid.p * Tu * df;

          SERIAL_ECHOLNPGM(STR_KU, Ku, STR_TU, Tu);

          SERIAL_ECHOLNPGM(STR_CLASSIC_PID);
          SERIAL_ECHOLNPGM(STR_KP, tune_pid.p, STR_KI, tune_pid.i, STR_KD, tune_pid.d);
        }
      }
      SHV((bias + d) >> 1);
      Serial.println(String(cycles) + " of " + String(ncycles));
      cycles++;
      minT = target;
    }

    // Did the temperature overshoot very far?
    if (current_temp > target + MAX_OVERSHOOT_PID_AUTOTUNE) {
      SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TEMP_TOO_HIGH);
      break;
    }

    // Report heater states every 2 seconds
    if (ELAPSED(ms, next_temp_ms)) {
      print_heater_state();
      next_temp_ms = ms + 2000UL;
    } // every 2 seconds

    // Timeout after PID_AUTOTUNE_MAX_CYCLE_MINS minutes since the last undershoot/overshoot cycle
    if ((ms - _MIN(t1, t2)) > MIN_TO_MS(PID_AUTOTUNE_MAX_CYCLE_MINS)) {
      SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TIMEOUT);
      break;
    }

    if (cycles > ncycles && cycles > 2) {
      SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_AUTOTUNE_FINISHED);

      // print resulting values
      SERIAL_ECHOLNPGM("Kp ", tune_pid.p);
      SERIAL_ECHOLNPGM("Ki ", tune_pid.i);
      SERIAL_ECHOLNPGM("Kd ", tune_pid.d);

      // set the result if chosen 
      if (set_result){
        _set_heater_pid(tune_pid); 
      }
    }
  }
  wait_for_heatup = false;

  SHV(0);

}
