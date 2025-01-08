#include <esp_now.h>
#include <WiFi.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
 
// Variables for test data
//int value_to_send;//testing data
const int FlexSensorPin = A2;
int FlexSensor;

Adafruit_MPU6050 mpu;
 
// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0xEC, 0x62, 0x60, 0x5A, 0x90, 0x54};
 
// Define a data structure
typedef struct Gesture {
  int pose;
} Gesture;
 // O:1      $
// Create a structured object
Gesture mode;
 
// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  
  // Set up Serial Monitor
  Serial.begin(115200);
  
  //trouble shooting steps
  
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);//set accelerometer range, 2,4,8,16
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);//set gyro range, 250, 500. 1000. 2000
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);//set filter bandwidth, 5, 10, 21, 44, 94, 184, 260
 
 //ESP NOW protocol starts here
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(FlexSensorPin,INPUT);

}
 
void loop() {
 
  // Gathering sensor data
  FlexSensor = analogRead(FlexSensorPin);
  Serial.print("FlexSensor:");
  Serial.println(FlexSensor);
  delay(50);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  

  Serial.println("X,Y,Z:");
  Serial.println(a.acceleration.x);
  Serial.println(a.acceleration.y);
  Serial.println(a.acceleration.z);
  

  if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=7) && (a.acceleration.y<=11)) && ((a.acceleration.z>=-3) && (a.acceleration.z<=3)) && FlexSensor<3500)
  {
    Serial.println("+x");
    mode.pose = 1;
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-11) && (a.acceleration.y<=-7)) && ((a.acceleration.z>=-3) && (a.acceleration.z<=3)) && FlexSensor<3500)
  {
    Serial.println("-x");
    mode.pose = 2;
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=-11) && (a.acceleration.z<=-6)) && FlexSensor<3500)
  {
    Serial.println("+y");
    mode.pose = 3;
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=7) && (a.acceleration.z<=11)) && FlexSensor<3500)
  {
    Serial.println("-y");
    mode.pose = 4;
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=-11) && (a.acceleration.z<=-6)) && FlexSensor>3500)
  {
    Serial.println("+z");
    mode.pose = 5;
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=7) && (a.acceleration.z<=11))&& FlexSensor>3500)
  {
    Serial.println("-z");
    mode.pose = 6;
  }
  else if (((a.acceleration.x>=7) && (a.acceleration.x<=11)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=-3) && (a.acceleration.z<=3))&& FlexSensor>3500)
  {
    Serial.println("pick");
    mode.pose = 7;
  }
  else
  {
    Serial.println("N");
    mode.pose = 0;
  }
  delay(50);
 
  //testing data
  //value_to_send = random(1,6); 
  //mode.pose = value_to_send
  
  //Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &mode, sizeof(mode));
   
  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending error");
  }
  delay(500);
}