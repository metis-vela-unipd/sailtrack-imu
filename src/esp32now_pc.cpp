#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
/*
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.print("Message received: ");
  Serial.println((char *)data);
}

void setup() {
  Serial.begin(115200);

  // Initialize Wi-Fi in Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(onReceive);
}

void loop() {
  // Do nothing, waiting for messages
}*/

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int accelT[3];
    int gyroT[3];
    int magT[3];
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);

  Serial.println("Accel: x, y, z  ");
  Serial.print(myData.accelT[0]);
  Serial.println(" x m/s^2");
  Serial.print(myData.accelT[1]);
  Serial.println(" y m/s^2");
  Serial.print(myData.accelT[2]);
  Serial.println(" z m/s^2");
  

  Serial.println("gyro: x, y, z  ");
  Serial.print(myData.gyroT[0]);
  Serial.println(" x rad/s");
  Serial.print(myData.gyroT[1]);
  Serial.println(" y rad/s");
  Serial.print(myData.gyroT[2]);
  Serial.println(" z rad/s");

  Serial.println("mag: x, y, z  ");
  Serial.print(myData.magT[0]);
  Serial.println(" x uT");
  Serial.print(myData.magT[1]);
  Serial.println(" y uT");
  Serial.print(myData.magT[2]);
  Serial.println(" z uT");


}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

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

}