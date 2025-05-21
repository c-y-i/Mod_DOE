/*
  Basic test of the Qwiic MicroPressure Sensor
  By: Alex Wende
  SparkFun Electronics
  Date: July 2020
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/16476
  
  This example demonstrates how to get started with the Qwiic MicroPressure Sensor board, and read pressures in various units.
*/

// Include the SparkFun MicroPressure library.
// Click here to get the library: http://librarymanager/All#SparkFun_MicroPressure

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SparkFun_MicroPressure.h>

// Customization options
uint8_t unit_code = 0; // 0 for kPa, 1 for PSI, add more as necessary - may be able to do this cleaner with a define for KPA/PSI
bool filter_errors = true; // filter negative values (generally caused by bad signal reads)
bool verbose = false; // send status success/failure to serial line

// Relevant MAC Addresses
uint8_t senderMAC1[] = {0xD8, 0xA0, 0x1D, 0x5D, 0xC7, 0xA8}; // PRE-Valve, COM8
uint8_t senderMAC2[] = {0x30, 0x83, 0x98, 0xEE, 0x4B, 0x3C}; // Post-Valve, COM12
uint8_t receiverMAC[] = {0xD8, 0xA0, 0x1D, 0x40, 0x74, 0x98}; 

// global atmospheric pressure [unit 'unit_code']
float atm = 0;

// global previous p value [unit 'unit_code']
float last_p = 0;

// Message Structure (send pressure)
typedef struct struct_message{
  float p_send;
} struct_message;

// Creat the struct_message
struct_message pData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (verbose){
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}

//Median Filter Setup
const int list_len = 7;
long int reading_counter = 0;
float pressure_list[] = {0,0,0,0,0,0,0};
float MedianFilter(float values[], int size) {
    float sortedValues[size];
    std::copy(values, values + size, sortedValues);
    std::sort(sortedValues, sortedValues + size);

    if (size % 2 == 1) {
        return sortedValues[size / 2];
    } else {
        int middle = size / 2;
        return (sortedValues[middle - 1] + sortedValues[middle]) / 2;
    }
}

/*
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */
//SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);
SparkFun_MicroPressure mpr; // Use default values with reset and EOC pins unused

void setup() {
  // Initalize UART, I2C bus, and connect to the micropressure sensor
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Wire.begin();

  if(!mpr.begin())
  {
    Serial.println("Cannot connect to MicroPressure sensor.");
    while(1);
  }

  switch (unit_code){
    case 0:
        for(int i=0;i<list_len;i++){
          pressure_list[i] = mpr.readPressure(KPA);
        }
      atm = MedianFilter(pressure_list, list_len);
      break;
    case 1:
        for(int i=0;i<list_len;i++){
          pressure_list[i] = mpr.readPressure(PSI);
        }
      atm = MedianFilter(pressure_list, list_len);
      break;
  }

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

    // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}

void loop() {
  /* The micropressure sensor outputs pressure readings in pounds per square inch (PSI).
     Optionally, if you prefer pressure in another unit, the library can convert the
     pressure reading to: pascals, kilopascals, bar, torr, inches of murcury, and
     atmospheres.
   */
  float this_p = 0;
  switch(unit_code){
    case 0:
      this_p = mpr.readPressure(KPA);
      break;
    case 1:
      this_p = mpr.readPressure(PSI);
      break;
  }


  pressure_list[reading_counter % 7] = this_p;
  reading_counter++;

  float gauge_p = MedianFilter(pressure_list, list_len)-atm;

  if (gauge_p < 0 && filter_errors){
    gauge_p = last_p;
  }

  long int start_time = millis();
  
  pData.p_send = gauge_p;
  Serial.print(gauge_p);
  switch(unit_code){
    case 0:
      Serial.println(" kPa");
      break;
    case 1:
      float this_p = mpr.readPressure(PSI);
      Serial.println(" PSI");
  }
  
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *) &pData, sizeof(pData));

  if (verbose) {
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }

  last_p = gauge_p;

  delay(3); // Prior code takes approx 7ms - this puts us at ~100Hz
}
