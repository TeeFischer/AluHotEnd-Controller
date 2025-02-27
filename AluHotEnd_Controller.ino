#include "max6675.h"  // by Adafruit
#include <PID_v1.h>   // by Brett Beauregard
#include "autotune.h"

//#include "pins_Uno.h"
#include "pins_Controllino_Maxi.h"

#define eStopTemp 500  // in Celsius

// MAX6675 objects for the thermocouples
MAX6675 thermocouple1(thermoSCK, thermoCS1, thermoSO);
//MAX6675 thermocouple2(thermoSCK, thermoCS2, thermoSO);
//MAX6675 thermocouple3(thermoSCK, thermoCS3, thermoSO);

// PID control variables
double T1, T1_Setpoint = 25.0, T1_Output;  // Setpoint for T1 temperature
double Kp = 10, Ki = 1, Kd = 0;  // PID tuning parameters
PID myPID(&T1, &T1_Output, &T1_Setpoint, Kp, Ki, Kd, DIRECT);

bool pidEnabled = false;  // Variable to track whether PID is enabled or not
#define autoTuneTarget 400
#define autoTuneCycles 4
#define autoTuneResult false

// Variables for time management
long int lastTime = 0;
long int thisTime = 0;
#define waitTime   250  //in ms

#if defined(CONTROLLINO_MAXI_AUTOMATION)
  #define eStopValue LOW
#else
  #define eStopValue HIGH
#endif

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);
  Serial.print("Starting...");
  
  // Wait to allow the MAX6675 to stabilize
  delay(500);

  // Initialize the PID controller in MANUAL mode (it will not control initially)
  myPID.SetMode(MANUAL);  // PID is initially disabled (manual mode)
  myPID.SetOutputLimits(0, 255);  // Set the output limits for PWM (0 to 255)
  myPID.SetSampleTime(1000);  // Set the sample time to 1 second
  pinMode(pwmPin, OUTPUT);  // Set the PWM pin as an output
  #if defined(CONTROLLINO_MAXI_AUTOMATION) 
    pinMode(eStopPin, INPUT);  // Set the PWM pin as an output
  #else
    pinMode(eStopPin, INPUT_PULLUP);  // Set the PWM pin as an output
  #endif

   Serial.println("Done!");
}

void loop() {
  // get current timestamp
  thisTime = millis();

  if (digitalRead(eStopPin) == eStopValue) {
    if(pidEnabled){
      Serial.println("E-Stop Triggered!");
    }
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
    else if (input == 't') {  // If 'r' is received, start the PID
      PID_autotune(autoTuneTarget, autoTuneCycles, autoTuneResult);
    }
  }

  // if enough time passed, enter this code block
  if(thisTime - lastTime > waitTime){
    // START OF WAITING CODE

    // Read temperatures from the thermocouples
    T1 = thermocouple1.readCelsius();
    float T2 = 0; // thermocouple2.readCelsius();
    float T3 = 0; // thermocouple3.readCelsius();

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

  if (pidEnabled){
    Serial.println("PID Stopped.");
    pidEnabled = false;
  }
  
}

void startPID() {
  myPID.SetMode(AUTOMATIC);  // Start PID (set to automatic mode)
  pidEnabled = true;
  Serial.println("PID Started.");
}
