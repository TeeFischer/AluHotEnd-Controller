#include "max6675.h"  // by Adafruit
#include <PID_v1.h>   // by Brett Beauregard
#include "autotune.h"

// setting, which pins to be used
//#include "pins_Uno.h"
#include "pins_Controllino_Maxi.h"

#define eStopTemp 800  // in Celsius

// autotune settings
#define autoTuneCycles 4      // number of heating cycles
#define autoTuneResult false  // automatically save the results of the autotuning

// PID control variables
double T1, T1_Setpoint = 400.0, T1_Output;  // Setpoint for T1 temperature
double T2=0, T3=0;
double Kp = 10, Ki = 1, Kd = 0;  // PID tuning parameters
PID myPID(&T1, &T1_Output, &T1_Setpoint, Kp, Ki, Kd, DIRECT);

bool pidEnabled = false;  // Variable to track whether PID is enabled or not

// Variables for time management
long int lastTime = 0;
long int thisTime = 0;
#define waitTime   250  //in ms

// MAX6675 objects for the thermocouples
MAX6675 thermocouple1(thermoSCK, thermoCS1, thermoSO);
//MAX6675 thermocouple2(thermoSCK, thermoCS2, thermoSO);
//MAX6675 thermocouple3(thermoSCK, thermoCS3, thermoSO);

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

  digitalWrite(CONTROLLINO_R0, HIGH);

  Serial.println("Done!");
}

void loop() {
  // get current timestamp
  thisTime = millis();

  // if the eStop is pressed, stop the PID Cycle and heaters
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
    else if (input == 't') {  // If 't' is received, start the autotuning
      if (pidEnabled){stopPID();}
      PID_autotune(T1_Setpoint, autoTuneCycles, autoTuneResult);
    }
    else if (input == 'z') {  // If 't' is received, set the target tmep
      setTargetTemp();
    }
  }

  // if enough time passed, enter this code block
  if(thisTime - lastTime > waitTime){
    // START OF WAITING CODE

    readTemps();

    // Output the current temperatures to the serial monitor
    printInfo();
    
    // If PID is enabled, compute the PID output and control PWM
    if (pidEnabled) {
      myPID.Compute();  // Calculate the new output for the PID control
      setHeaterValues(T1_Output);
    }

    if ( T1 > eStopTemp){
      stopPID();
    }

    // END OF WAITING CODE
    // update lastTime
    lastTime = thisTime;
  }
}

float readTemps(){
  // Read temperatures from the thermocouples
  T1 = thermocouple1.readCelsius();
  // T2 = thermocouple2.readCelsius();
  // T3 = thermocouple3.readCelsius();

  return float(T1);
}

void setHeaterValues(uint8_t _value){
  T1_Output = _value;
  analogWrite(pwmPin, _value);  

  #if defined(CONTROLLINO_MAXI_AUTOMATION) 
  digitalWrite(CONTROLLINO_R8, HIGH);  // activate heater relays
  digitalWrite(CONTROLLINO_R9, HIGH);
  #endif
}

// Output the current temperatures to the serial monitor
void printInfo(){
  // Erstelle einen String mit den Werten und sende ihn auf einmal
    String output = "T1:" + String(T1) + 
    " °C T2:" + String(T2) +
    " °C T3:" + String(T3) +
    " °C PWM:" + String(T1_Output) +
    " Time:" + String(millis());
    
    Serial.println(output);
}

// This function stops the PID calculation and shuts off the heater
void stopPID() {
  myPID.SetMode(MANUAL);  // Stop PID (set to manual mode)

  // reset the PID-Output value and write the output
  T1_Output = 0;
  analogWrite(pwmPin, T1_Output);

  #if defined(CONTROLLINO_MAXI_AUTOMATION) 
  digitalWrite(CONTROLLINO_R8, LOW);  // deactivate heater relays
  digitalWrite(CONTROLLINO_R9, LOW);
  #endif

  // print the status once
  if (pidEnabled){
    Serial.println("PID Stopped.");
    pidEnabled = false;
  } 
}

void startPID() {
  myPID.SetMode(AUTOMATIC);  // Start PID (set to automatic mode)
  pidEnabled = true;

  #if defined(CONTROLLINO_MAXI_AUTOMATION) 
  digitalWrite(CONTROLLINO_R8, HIGH);  // activate heater relays
  digitalWrite(CONTROLLINO_R9, HIGH);
  #endif

  Serial.println("PID Started. Target=" + String(T1_Setpoint));
}

void setTargetTemp(){
  float input = 0;
  Serial.println("Enter target temperature (in °C):");
  
  while(true){
    if (Serial.available() > 0) {
      input = Serial.parseFloat();
    }
    if(input != 0){
      break;
    }
  }

  if(input == 0){
    Serial.println("Error: No target was entered!");
  }
  else if(input > eStopTemp){
    input = eStopTemp;
    Serial.println("You entered " + String(input) + " °C but eStopTemp is " + String(eStopTemp) + " °C!");
  }
  // Gib die Eingabe aus
  Serial.print("Target set to: ");
  Serial.println(input);

  T1_Setpoint = float(input);
}
