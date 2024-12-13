
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <iostream>
#include <string>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>

#define I2C_SDA_PIN 27
#define I2C_SCL_PIN 25

// Create the LSM9DS1 object
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Sensor_Calibration_EEPROM cal;

// I2C address definitions (default for LSM9DS1 breakout)
#define LSM9DS1_SCK  0x6B  // Accelerometer + gyroscope
#define LSM9DS1_MCK  0x1E  // Magnetometer


uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x19, 0x5E, 0x14};

int loopcount = 0;
float offsets[16];
// 1=raw, 2=cal1, 3=cal2

// Structure example to send data
// Must match the receiver structure
typedef struct raw_message {
  int id;
  int accelT[3];
  int gyroT[3];
  int magT[3];

} raw_message;

typedef struct cal1_message
{
  int id;
  double accelT[3];
  double gyroT[3];
  double magT[4];
} cal1_message;

typedef struct cal2_message
{
  int id;
  double softiron[9];
} cal2_message;



void receiveCalibration() {
    cal.accel_zerog[0] = offsets[0];
    cal.accel_zerog[1] = offsets[1];
    cal.accel_zerog[2] = offsets[2];

    cal.gyro_zerorate[0] = offsets[3];
    cal.gyro_zerorate[1] = offsets[4];
    cal.gyro_zerorate[2] = offsets[5];

    cal.mag_hardiron[0] = offsets[6];
    cal.mag_hardiron[1] = offsets[7];
    cal.mag_hardiron[2] = offsets[8];

    cal.mag_field = offsets[9];

    cal.mag_softiron[0] = offsets[10];
    cal.mag_softiron[1] = offsets[13];
    cal.mag_softiron[2] = offsets[14];
    cal.mag_softiron[3] = offsets[13];
    cal.mag_softiron[4] = offsets[11];
    cal.mag_softiron[5] = offsets[15];
    cal.mag_softiron[6] = offsets[14];
    cal.mag_softiron[7] = offsets[15];
    cal.mag_softiron[8] = offsets[12];

    cal.saveCalibration();

    //ToDo: add a led for notifying the calibiration status !!!
  
}

double power(int base, int exponent) {
    double result = 1;
    for (int i = 0; i < exponent; ++i) {
        result *= base;
    }
    return result;
}

// Create a struct_message called myData
raw_message myData;
cal1_message cal1Msg;
cal2_message cal2Msg;

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
 
  //while (!Serial);  uncomment if code not working
  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);

  cal.begin(); 
  cal.loadCalibration();

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
  Wire.setClock(400000);

}

void loop() {
  esp_err_t result;
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
  
  
  
  
  loopcount++;

  myData.id=0;
  cal1Msg.id=0;
  cal2Msg.id=0;



  if (loopcount == 50 || loopcount > 100) {
        cal1Msg.id=2;
        for (int i = 0; i < 3; i++) {
            cal1Msg.accelT[i]=cal.accel_zerog[i];
        }
        for (int i = 0; i < 3; i++) {
            cal1Msg.gyroT[i]=cal.gyro_zerorate[i];
        }  
        for (int i = 0; i < 3; i++) {
            cal1Msg.magT[i]=cal.mag_hardiron[i];
        }  
        
        cal1Msg.magT[3]=cal.mag_field;

        esp_now_send(broadcastAddress, (uint8_t *) &cal1Msg, sizeof(cal1Msg));

        loopcount++;
    }
  else if (loopcount >= 100) {
        cal2Msg.id=3;
        for (int i = 0; i < 9; i++) {
          cal2Msg.softiron[i]=cal.mag_softiron[i];
        }

        esp_now_send(broadcastAddress, (uint8_t *) &cal2Msg, sizeof(cal2Msg));
        
        loopcount = 0;
    }
  else{
      myData.id=1;
      myData.accelT[0]=accel.acceleration.x * 8192 / SENSORS_GRAVITY_STANDARD;
      myData.accelT[1]=accel.acceleration.y * 8192 / SENSORS_GRAVITY_STANDARD;
      myData.accelT[2]=accel.acceleration.z * 8192 / SENSORS_GRAVITY_STANDARD;

      myData.gyroT[0]=gyro.gyro.x * SENSORS_RADS_TO_DPS * 16;
      myData.gyroT[1]=gyro.gyro.y * SENSORS_RADS_TO_DPS * 16;
      myData.gyroT[2]=gyro.gyro.z * SENSORS_RADS_TO_DPS * 16;

      myData.magT[0]=mag.magnetic.x * 10;
      myData.magT[1]=mag.magnetic.y * 10;
      myData.magT[2]=mag.magnetic.z * 10;

      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    }
  Serial.println(myData.id);
  Serial.println(cal1Msg.id);
  Serial.println(cal2Msg.id);



  


 // Send message via ESP-NOW
  //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
 /* if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }*/

  delay(10);
}

 


