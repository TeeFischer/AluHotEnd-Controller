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
  
  Serial.print("C = "); 
  Serial.println(thermocouple1.readCelsius());
 
  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(1000);
}
