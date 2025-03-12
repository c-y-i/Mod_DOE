#include "HX711.h"
#include <esp_now.h>
#include <WiFi.h>


//Header
HX711 scale;

const uint8_t LOADCELL_DOUT_PIN = 22;
const uint8_t LOADCELL_SCK_PIN = 21;
long init_time = 0;

// const float calibration_factor = 48450; //7050 or 48450 in the past -GMC
// const float calibration_factor = 9222.8; //104
const float calibration_factor = 9109.6; //105


void setup() {
  Serial.begin(115200);

  // Initiate scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  scale.set_scale(calibration_factor);
  // Serial.println(calibration_factor);

  init_time = millis();

}

void loop() {
  //start timer
  long start_time = millis();

  // Read scale
  float This_Reading = scale.get_units(); //N (??)

  // Print values
  long this_time = millis() - init_time;
  Serial.print(this_time); // ms
  Serial.print(", ");
  Serial.print(This_Reading); // lbs
  Serial.print("\n"); 

  //check timer, delay to keep @ 50Hz (or as close as possible - for 25hz 41 seems to work better than 40?)
  int new_delay = 21 - (millis()-start_time);

  if (new_delay > 0 ) {
    delay(new_delay);
  }
}
