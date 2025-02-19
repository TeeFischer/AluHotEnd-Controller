#include "max6675.h"
#include <PID_v1.h>

// Pin definitions
// thermoelement modules Pin
#define thermoSO  5
#define thermoSCK 6
#define thermoCS1 2
#define thermoCS2 3
#define thermoCS3 4

// PWM Pin
#define pwmPin 9

// eStop Pin
#define eStopPin 8
#define eStopTemp 500  // in Celsius

// MAX6675 objects for the thermocouples
MAX6675 thermocouple1(thermoSCK, thermoCS1, thermoSO);
MAX6675 thermocouple2(thermoSCK, thermoCS2, thermoSO);
MAX6675 thermocouple3(thermoSCK, thermoCS3, thermoSO);

// PID control variables
double T1, T1_Setpoint = 25.0, T1_Output;  // Setpoint for T1 temperature
double Kp = 10, Ki = 1, Kd = 0;  // PID tuning parameters
PID myPID(&T1, &T1_Output, &T1_Setpoint, Kp, Ki, Kd, DIRECT);

bool pidEnabled = false;  // Variable to track whether PID is enabled or not

// Variables for time management
long int lastTime = 0;
long int thisTime = 0;
#define waitTime   250  //in ms

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);
  
  // Wait to allow the MAX6675 to stabilize
  delay(500);

  // Initialize the PID controller in MANUAL mode (it will not control initially)
  myPID.SetMode(MANUAL);  // PID is initially disabled (manual mode)
  myPID.SetOutputLimits(0, 255);  // Set the output limits for PWM (0 to 255)
  myPID.SetSampleTime(1000);  // Set the sample time to 1 second
  pinMode(pwmPin, OUTPUT);  // Set the PWM pin as an output

  pinMode(eStopPin, INPUT_PULLUP);  // Set the PWM pin as an output
}

void loop() {
  // get current timestamp
  thisTime = millis();

  if (digitalRead(eStopPin) == 0) {
    stopPID();
  }

  // Toggle PID controller state based on serial input or condition
  if (Serial.available()) {
    char input = Serial.read();  // Read the input character
    if (input == 's') {  // If 's' is received, stop the PID
      stopPID();
    } 
    else if (input == 'a') {  // If 'r' is received, start the PID
      startPID();
    }
  }

  // if enough time passed, enter this code block
  if(thisTime - lastTime > waitTime){
    // START OF WAITING CODE

    // Read temperatures from the thermocouples
    T1 = thermocouple1.readCelsius();
    float T2 = thermocouple2.readCelsius();
    float T3 = thermocouple3.readCelsius();

    // Output the current temperatures to the serial monitor
    Serial.println("T1:" + String(T1) + "°C T2:" + String(T2) + "°C T3:" + String(T3) + "°C PWM:" + String(T1_Output));
    
    // If PID is enabled, compute the PID output and control PWM
    if (pidEnabled) {
      myPID.Compute();  // Calculate the new output for the PID control
      analogWrite(pwmPin, T1_Output);  // Set the PWM signal based on PID output
    }

    if ( T1 > eStopTemp){
      stopPID();
    }

    // END OF WAITING CODE
    // update lastTime
    lastTime = thisTime;
  }
}

void stopPID() {
  myPID.SetMode(MANUAL);  // Stop PID (set to manual mode)

  // reset the PID value and set the output
  T1_Output = 0;
  analogWrite(pwmPin, T1_Output);

  pidEnabled = false;
  Serial.println("PID Stopped.");
}

void startPID() {
  myPID.SetMode(AUTOMATIC);  // Start PID (set to automatic mode)
  pidEnabled = true;
  Serial.println("PID Started.");
}
