#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
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

// Create a struct_message called myData
raw_message myData;
cal1_message cal1Msg;
cal2_message cal2Msg;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  memcpy(&cal1Msg, incomingData, sizeof(cal1Msg));
  memcpy(&cal2Msg, incomingData, sizeof(cal2Msg));
  if(myData.id==1) 
  {
    Serial.print("Raw:");

    for(int i=0;i<3;i++)
    {
      Serial.print(myData.accelT[i]);
      Serial.print(",");
    }

    for(int i=0;i<3;i++)
    {
      Serial.print(myData.gyroT[i]);
      Serial.print(",");

    }

    for(int i=0;i<2;i++)
    {
      Serial.print(myData.magT[i]);
      Serial.print(",");
    }
    Serial.print(myData.magT[2]);
    Serial.println();
  }

  else if (cal1Msg.id==2)
  {
    Serial.print("Cal1:");

    for(int i=0;i<3;i++)
    {
      Serial.print(cal1Msg.accelT[i]);
      Serial.print(",");
    }

    for(int i=0;i<3;i++)
    {
      Serial.print(cal1Msg.gyroT[i]);
      Serial.print(",");
    }

    for(int i=0;i<3;i++)
    {
      Serial.print(cal1Msg.magT[i]);
      Serial.print(",");
    }
    Serial.print(cal1Msg.magT[3]);
    Serial.println();
  }

  else if(cal2Msg.id==3)
  {
    Serial.print("Cal2:");
    for(int i=0;i<8;i++)
    {
      Serial.print(cal2Msg.softiron[i]);
      Serial.print(",");
    }
    Serial.print(cal2Msg.softiron[8]);
    Serial.println();
  }
  
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
  
}
 
void loop() {
  esp_now_register_recv_cb(OnDataRecv);
}