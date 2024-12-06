#include <Arduino.h>
#include <SailtrackModule.h>

#define SENDING_TOPIC   "sensor/motioncal_pc"
#define RECEIVING_TOPIC "sensor/motioncal_imu"

SailtrackModule stm;

byte calData[68];
byte calCount = 0;

uint16_t crc16Update(uint16_t crc, uint8_t a) {
    int i;
    crc ^= a;
    for (i = 0; i < 8; i++) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc = (crc >> 1);
    }
    return crc;
}

void receiveCalibration() {
    uint16_t crc;
    byte b, i;

    while (Serial.available()) {
        b = Serial.read();
        if (calCount == 0 && b != 117)
            return;
        if (calCount == 1 && b != 84) {
            calCount = 0;
            return;
        }

        calData[calCount++] = b;
        if (calCount < 68)
            return;

        crc = 0xFFFF;
        for (i = 0; i < 68; i++)
            crc = crc16Update(crc, calData[i]);

        if (!crc) {
            float offsets[16];
            memcpy(offsets, calData + 2, 16 * 4);
        
            StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
            JsonArray offsets_json = doc.createNestedArray("offsets_json");

            for (float value : offsets) {
                offsets_json.add(value);
            }

            stm.publish(SENDING_TOPIC, doc.as<JsonObjectConst>());
        
            calCount = 0;
            return;
        }

        for (i = 2; i < 67; i++) {
            if (calData[i] == 117 && calData[i + 1] == 84) {
                calCount = 68 - i;
                memmove(calData, calData + i, calCount);
                return;
            }
        }

        if (calData[67] == 117) {
            calData[0] = 117;
            calCount = 1;
        }
        else
            calCount = 0;        
    }
}

void print_JsonArray(JsonArray a, String name) {
    Serial.print(name + ":");
    size_t arraySize = a.size();
    size_t index = 0;

    // Handle the different number of significant digits
    int num_digits = 3;
    if (name == "Raw")
        num_digits = 0;
    else if (name == "Cal2")
        num_digits = 4;

    for (JsonVariant value : a) {
        Serial.print(value.as<double>(), num_digits);

        if (index < arraySize - 1) {
            Serial.print(",");
        }

        index++;
    }
    Serial.println("");
}

class ModuleCallbacks : public SailtrackModuleCallbacks {
    void onMqttMessage(const char *topic, JsonObjectConst payload) {
        char output[STM_MQTT_DATA_BUFFER_SIZE];
        serializeJson(payload, output);
        StaticJsonDocument<STM_JSON_DOCUMENT_BIG_SIZE> doc;
        deserializeJson(doc, output);
        if (payload.containsKey("Raw")) {
            JsonArray array = doc["Raw"].as<JsonArray>();
            print_JsonArray(array, "Raw");
        }
        else if (payload.containsKey("Cal1")) {
            JsonArray array = doc["Cal1"].as<JsonArray>();
            print_JsonArray(array, "Cal1");
        }
        else if (payload.containsKey("Cal2")) {
            JsonArray array = doc["Cal2"].as<JsonArray>();
            print_JsonArray(array, "Cal2");
        }
    }
};

void setup() {
    Serial.begin(115200);
    stm.begin("motioncal_pc", IPAddress(192, 168, 42, 108), new ModuleCallbacks());
    stm.subscribe(RECEIVING_TOPIC);
    
}

void loop() {
    receiveCalibration();
    delay(100);
}
