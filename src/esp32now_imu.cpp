
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <iostream>
#include <string>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
//#include <Adafruit_Sensor_Calibration.h>
//#include <SPI.h>
int counter_global=0;
// Create the LSM9DS1 object
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// I2C address definitions (default for LSM9DS1 breakout)
#define LSM9DS1_SCK  0x6B  // Accelerometer + gyroscope
#define LSM9DS1_MCK  0x1E  // Magnetometer

//parte per comunicare

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x19, 0x5E, 0x14};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {

int accelT[3];
int gyroT[3];
int magT[3];
int counter;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//fine parte per comunicare

void setup() {
  
  // Init Serial Monitor
  Serial.begin(115200);
 
  while (!Serial);
  Wire.begin(27, 25);

  // Initialize I2C communication with the LSM9DS1
  if (!lsm.begin()) {
    Serial.println("Failed to find LSM9DS1 chip");
    while (1);
  }
  Serial.println("LSM9DS1 initialized!");

  // Set ranges and sampling rates if desired
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  // ±2g
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); // ±245 degrees per second
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   // ±4 gauss
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
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



//parte 2
  while (!Serial);
  Wire.begin(27, 25);

  // Initialize I2C communication with the LSM9DS1
  if (!lsm.begin()) {
    Serial.println("Failed to find LSM9DS1 chip");
    while (1);
  }
  Serial.println("LSM9DS1 initialized!");

  // Set ranges and sampling rates if desired
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  // ±2g
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); // ±245 degrees per second
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   // ±4 gauss
}

void loop() {
  // Read accelerometer data
  sensors_event_t accel, gyro, mag, temp;
  lsm.read();
  lsm.getEvent( &accel, &gyro, &mag, &temp);

  // Print accelerometer data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2, ");
  Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2, ");
  Serial.print("Z: "); Serial.println(accel.acceleration.z); Serial.println(" m/s^2");

  // Print gyroscope data
  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" rad/s, ");
  Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" rad/s, ");
  Serial.print("Z: "); Serial.println(gyro.gyro.z); Serial.println(" rad/s");

  // Print magnetometer data
  Serial.print("Mag X: "); Serial.print(mag.magnetic.x); Serial.print(" uT, ");
  Serial.print("Y: "); Serial.print(mag.magnetic.y); Serial.print(" uT, ");
  Serial.print("Z: "); Serial.println(mag.magnetic.z); Serial.println(" uT");
  
  counter_global++;
  // Small delay for readability

myData.accelT[0]=accel.acceleration.x;
myData.accelT[1]=accel.acceleration.y;
myData.accelT[2]=accel.acceleration.z;

myData.gyroT[0]=gyro.gyro.x;
myData.gyroT[1]=gyro.gyro.y;
myData.gyroT[2]=gyro.gyro.z;

myData.magT[0]=mag.magnetic.x;
myData.magT[1]=mag.magnetic.y;
myData.magT[2]=mag.magnetic.z;

myData.counter=counter_global;

 // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(1000);
}

 


