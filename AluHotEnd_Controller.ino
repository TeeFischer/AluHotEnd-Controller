#include "max6675.h"
#include <PID_v1.h>

// Pin definitions
int thermoDO = 5;
int thermoCLK = 6;
int thermoCS1 = 2;
int thermoCS2 = 3;
int thermoCS3 = 4;

// MAX6675 objects for the thermocouples
MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
MAX6675 thermocouple3(thermoCLK, thermoCS3, thermoDO);

// PID control variables
double T1, T1_Setpoint = 25.0, T1_Output;  // Setpoint for T1 temperature
double Kp = 2, Ki = 5, Kd = 1;  // PID tuning parameters
PID myPID(&T1, &T1_Output, &T1_Setpoint, Kp, Ki, Kd, DIRECT);

// PWM Pin
int pwmPin = 9;

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);
  
  // Wait to allow the MAX6675 to stabilize
  delay(500);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);  // Set PID mode to AUTOMATIC
  myPID.SetOutputLimits(0, 255);  // Set the output limits for PWM (0 to 255)
  myPID.SetSampleTime(1000);  // Set the sample time to 1 second
  pinMode(pwmPin, OUTPUT);  // Set the PWM pin as an output
}

void loop() {
  // Read temperatures from the thermocouples
  T1 = thermocouple1.readCelsius();
  float T2 = thermocouple2.readCelsius();
  float T3 = thermocouple3.readCelsius();

  // Output the current temperatures to the serial monitor
  Serial.println("T1: " + String(T1) + "°C T2: " + String(T2) + "°C T3: " + String(T3) + "°C");
  
  // Compute the PID output for T1
  myPID.Compute();  // Calculate the new output for the PID control
  
  // Set the PWM signal on pin 9 based on the PID output
  analogWrite(pwmPin, T1_Output);
  
  // Wait to get the next temperature reading
  delay(250);  // MAX6675 requires at least 250ms between readings
}
