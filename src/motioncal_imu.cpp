#include <Arduino.h>
#include <SPI.h>
#include <SailtrackModule.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor_Calibration.h>

#define I2C_SDA_PIN 27
#define I2C_SCL_PIN 25
#define STM_NOTIFICATION_LED_PIN 19
#define SENDING_TOPIC   "sensor/motioncal_imu"
#define RECEIVING_TOPIC "sensor/motioncal_pc"

SailtrackModule stm;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Sensor_Calibration_EEPROM cal;

int loopcount = 0;
float offsets[16];

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

    StaticJsonDocument<STM_JSON_DOCUMENT_SMALL_SIZE> doc;
    JsonArray array = doc.createNestedArray("Result");
    array.add("Calibration saved successfully");
    stm.publish("sensor/Result", doc.as<JsonObjectConst>());   
}

class ModuleCallbacks : public SailtrackModuleCallbacks {
    void onMqttMessage(const char *topic, JsonObjectConst payload) {
        StaticJsonDocument<STM_JSON_DOCUMENT_BIG_SIZE> doc;
        JsonArrayConst array = payload["offests_json"];
        int i = 0;
        for (JsonVariantConst value : array) {
            offsets[i++]=value.as<float>();
        }
        receiveCalibration();
    } 
};

double power(int base, int exponent) {
    double result = 1;
    for (int i = 0; i < exponent; ++i) {
        result *= base;
    }
    return result;
}

void add_to_JsonArray(JsonArray cal, float a[], int length, int digits) {
    double k = power(10, digits);
    for (size_t i = 0; i < length; i++) {
        cal.add(round(a[i] * (int)k) / k); // only "digits" decimal digits
    }
}

void setup(void) {
    stm.begin("motioncal_imu", IPAddress(192, 168, 42, 102), new ModuleCallbacks());
    stm.subscribe(RECEIVING_TOPIC);
    Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    cal.begin();
    cal.loadCalibration();
    lsm.begin();
	lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  	lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  	lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    Wire.setClock(400000);
}

void loop() {
    sensors_event_t accelEvent, gyroEvent, magEvent, tempEvent;

    lsm.read();
    lsm.getEvent(&accelEvent, &magEvent, &gyroEvent, &tempEvent);

    StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
    JsonArray Raw = doc.createNestedArray("Raw");

    Raw.add(int(accelEvent.acceleration.x * 8192 / SENSORS_GRAVITY_STANDARD));
    Raw.add(int(accelEvent.acceleration.y * 8192 / SENSORS_GRAVITY_STANDARD));
    Raw.add(int(accelEvent.acceleration.z * 8192 / SENSORS_GRAVITY_STANDARD));
    Raw.add(int(gyroEvent.gyro.x * SENSORS_RADS_TO_DPS * 16));
    Raw.add(int(gyroEvent.gyro.y * SENSORS_RADS_TO_DPS * 16));
    Raw.add(int(gyroEvent.gyro.z * SENSORS_RADS_TO_DPS * 16));
    Raw.add(int(magEvent.magnetic.x * 10));
    Raw.add(int(magEvent.magnetic.y * 10));
    Raw.add(int(magEvent.magnetic.z * 10));

    stm.publish(SENDING_TOPIC, doc.as<JsonObjectConst>());

    loopcount++;

    if (loopcount == 50 || loopcount > 100) {
        
        StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
        JsonArray Cal1 = doc.createNestedArray("Cal1");

        add_to_JsonArray(Cal1, cal.accel_zerog, 3, 3);

        add_to_JsonArray(Cal1, cal.gyro_zerorate, 3, 3);

        add_to_JsonArray(Cal1, cal.mag_hardiron, 3, 3);

        Cal1.add(round(cal.mag_field * power(10, 3)) / power(10, 3));

        stm.publish(SENDING_TOPIC, doc.as<JsonObjectConst>());

        loopcount++;
    }
    if (loopcount >= 100) {
        StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
        JsonArray Cal2 = doc.createNestedArray("Cal2");

        add_to_JsonArray(Cal2, cal.mag_softiron, 9, 4);

        stm.publish(SENDING_TOPIC, doc.as<JsonObjectConst>());

        loopcount = 0;
    }

    delay(10);
}
