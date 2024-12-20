#include <WiFi.h>

void setup() {
  // Start Serial communication
  Serial.begin(115200);
  while (!Serial);

  // Set WiFi mode to station (STA)
  WiFi.mode(WIFI_STA);

  // Get and print the MAC address
  String macAddress = WiFi.macAddress();
  Serial.print("ESP32 MAC Address: ");
  Serial.println(macAddress);
}

void loop() {
  // Do nothing in the loop
}
