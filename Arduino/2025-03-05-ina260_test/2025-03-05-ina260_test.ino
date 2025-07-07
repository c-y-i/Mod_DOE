#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();
long start_time = 0;

void setup() {
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) { delay(10); }

  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  start_time = millis();
  Serial.println("Found INA260 chip");
}

void loop() {
  long this_time = millis();
  // Serial.print("Current: ");
  Serial.print(this_time - start_time);
  Serial.print(", ");
  Serial.println(ina260.readCurrent()); //mA
  // Serial.println(" mA");
// 
//  Serial.print("Bus Voltage: ");
//  Serial.print(ina260.readBusVoltage());
//  Serial.println(" mV");

  //Serial.print("Power: ");
  //Serial.print(ina260.readPower());
  //Serial.println(" mW");

  //Serial.println();
  delay(5);
}
