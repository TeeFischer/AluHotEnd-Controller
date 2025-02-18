// this example is public domain. enjoy! https://learn.adafruit.com/thermocouple/
// downloaded from https://randomnerdtutorials.com/arduino-k-type-thermocouple-max6675/ 

#include "max6675.h"

int thermoDO = 5;
int thermoCLK = 6;

int thermoCS1 = 2;
int thermoCS2 = 3;
int thermoCS3 = 4;

MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
MAX6675 thermocouple3(thermoCLK, thermoCS3, thermoDO);

void setup() {
  Serial.begin(115200);

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp
  
  float T1 = thermocouple1.readCelsius();
  float T2 = thermocouple2.readCelsius();
  float T3 = thermocouple3.readCelsius();

  Serial.println("T1:" + String(T1) + "°C T2:" + String(T2) + "°C T3:" + String(T3) + "°C");
 
  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(250);
}
