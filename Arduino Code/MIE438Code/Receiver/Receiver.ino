#include <esp_now.h>
#include <WiFi.h>
 
// Define a data structure
typedef struct Gesture {
  int pose;
} Gesture;
 
// Create a structured object
Gesture mode;
 
 
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&mode, incomingData, sizeof(mode));
  //Serial.print("Data received: ");
  //Serial.println(len);
  Serial.print("O:");
  Serial.print(mode.pose);
  Serial.print("      $");
}

void setup() {
  // put your setup code here, to run once:
  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // put your main code here, to run repeatedly:

}
