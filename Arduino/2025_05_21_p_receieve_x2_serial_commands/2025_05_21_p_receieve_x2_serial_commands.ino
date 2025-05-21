/*
  Gregory M. Campbell
  04-24-2025

  Adapted from: https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
*/

#include <esp_now.h>
#include <WiFi.h>

#define AIR_PUMP_ON 'w'
#define AIR_PUMP_OFF 's'

const uint8_t PUMP_PIN = 14;

// PUMP PWM
uint8_t PWM_Ch = 0; // 0-15 
uint8_t PWM_Freq = 100; // Arbitrary, ajust as necessary
uint8_t PWM_Res = 8; // Sets 0-255
const int Pump_Value = 255; // 0-255, equivalent to duty cycle
// const int Pump_Value = 100;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message{
  float p_send; 
} struct_message;

// Init values for input pressures
float Sent_Pressure1 = 0;
float Sent_Pressure2 = 0;

// Set loop frequency (i.e. 25Hz)
const int loop_period = 40; // Set frequency here

uint8_t senderMAC1[] = {0xD8, 0xA0, 0x1D, 0x5D, 0xC7, 0xA8}; // PRE-Valve, COM
uint8_t senderMAC2[] = {0x30, 0x83, 0x98, 0xEE, 0x4B, 0x3C}; // Post-Valve, 
uint8_t receiverMAC[] = {0xD8, 0xA0, 0x1D, 0x40, 0x74, 0x98}; // COM6

// Create a struct_message called myData
struct_message myData;
uint8_t MAC_in;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  memcpy(&MAC_in, mac, sizeof(mac));
  if(MAC_in==48){Sent_Pressure2 = myData.p_send;} // Needs to be verified.
  if(MAC_in==216){Sent_Pressure1 = myData.p_send;} // Value for Initial MAC
}
 
void setup() {
  // Initialize Serial Monitor
  // Serial.begin(115200);
  Serial.begin(500000);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // PWM Pump PIN
  ledcSetup(PWM_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(PUMP_PIN, PWM_Ch);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

    //start timer
  long int start_time = millis();
  Serial.print(Sent_Pressure1, 4); // kPa
  Serial.print(", ");
  Serial.print(Sent_Pressure2, 4); // kPa
  Serial.print("\n");

    //check timer, delay to keep @ 10Hz (or as close as possible)
  int new_delay = loop_period - (millis()-start_time);
  int loop_start = millis();

  if(new_delay > 0){
    while (millis()-loop_start < new_delay ) {
        Check_Events();
    }
  }
  else{Check_Events();}

}

void Clear_Serial(){
      while(Serial.available()){  //clear the inputs
        char temp = Serial.read();
        delay(5);
      }
}

void Check_Events(){
  //Check if Serial is Available
  if(Serial.available()){
       char Read_Char = Serial.read();
       Clear_Serial(); //NOTE: Only allows one character at a time. May have to change in the future.
       if(Read_Char == '\n'){
        //This is an enter 'key'. Do Nothing.
       }
       else{
        switch (Read_Char){
          case AIR_PUMP_ON: 
            ledcWrite(PWM_Ch, Pump_Value);
            return;
          case AIR_PUMP_OFF:
            ledcWrite(PWM_Ch, 0);
            return;
          } 
       }
  }
  delay(2);
}
