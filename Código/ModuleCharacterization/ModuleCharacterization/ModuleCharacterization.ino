#include "CNC_Shield_UNO.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include "SparkFun_ADS122C04_ADC_Arduino_Library.h" // Modded library for compatibility with Helios project

#define HELIOS_ADDR 0x44
#define IMU_0_ADDR 0x08
#define IMU_1_ADDR 0x68

SFE_ADS122C04 heliosSensor;
      
sensors_event_t orientationData;
float h0 = 0, h1 = 0, h2 = 0, h3 = 0; // Helios module measurement
float qx_0=0, qy_0=0, qz_0=0; // IMU_0 Euler angles
float qx_1=0, qy_1=0, qz_1=0; // IMU_1 Euler angles
float qx_1_ini=0, qy_1_ini=0, qz_1_ini=0; // IMU_1 Euler angles initial

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;  // Delay between samples

Adafruit_BNO055 bno_0 = Adafruit_BNO055(55, IMU_0_ADDR, &Wire);
Adafruit_BNO055 bno_1 = Adafruit_BNO055(55, IMU_1_ADDR, &Wire);

//#define ACTUATOR_BOARD

void setup() {
  #ifdef ACTUATOR_BOARD
    setupCNC();
    pinMode(HOLD, OUTPUT);
    randomSeed(millis());

  #else
    Serial.begin(115200);
    while (!Serial) delay(10);  // Wait for serial port to open
    Wire.begin();


    // HELIOS INIT
    while(!heliosSensor.begin(HELIOS_ADDR)){
      Serial.println(F("Helios Module not detected. Retrying..."));
      delay(500);
    }

    heliosSensor.configureADCmode(ADS122C04_RAW_MODE);
    heliosSensor.setGain(ADS122C04_GAIN_4);

    // IMU INIT
    while(!bno_0.begin()){
      Serial.print("BNO055 not detected. Retrying...");
      delay(500);
    }

    int eeAddress = 0;
    long bnoID;

    EEPROM.get(eeAddress, bnoID);
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;
    
    bno_0.getSensor(&sensor);
    if (bnoID != sensor.sensor_id){
        Serial.println("\nNo Calibration Data for BNO055 with address ");
        Serial.println(IMU_0_ADDR, HEX);
        delay(500);
    }
    else
    {
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        bno_0.setSensorOffsets(calibrationData);
        Serial.print("\n\nCalibration data loaded into BNO055 with address ");
        Serial.println(IMU_0_ADDR, HEX);
    }

    // IMU INIT
    while(!bno_1.begin()){
      Serial.print("BNO055 not detected. Retrying...");
      delay(500);
    }
      
    eeAddress = 0;
    bno_1.getSensor(&sensor);
    if (bnoID != sensor.sensor_id){
        Serial.println("\nNo Calibration Data for BNO055 with address ");
        Serial.println(IMU_1_ADDR, HEX);
        delay(500);
    }
    else
    {
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        bno_1.setSensorOffsets(calibrationData);
        Serial.print("\n\nCalibration data loaded into BNO055 with address ");
        Serial.println(IMU_1_ADDR, HEX);
    }
    pinMode(A1, INPUT);
    Serial.println("Theta, Phi, h0, h1, h2, h3");
    bno_1.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    qx_1 = orientationData.orientation.x;
    qy_1 = orientationData.orientation.y;
    qz_1 = orientationData.orientation.z;
  #endif
}

void loop() {
  #ifdef ACTUATOR_BOARD
    long n[4];
    long _n[4];
    CoordsPCC coords = {PI/4.0, 0};
    digitalWrite(HOLD, HIGH);
    for(coords.phi=0; coords.phi<(2*PI-PI/6.0); coords.phi+=PI/6.0){
      for(uint8_t i=0;  i<4; ++i){
        n[i] = length2steps(cableIKine(coords, i));
        _n[i] = -n[i];
      }
      digitalWrite(MOT_EN, LOW);
      stepParallel(n);
      digitalWrite(MOT_EN, HIGH);

      digitalWrite(MOT_EN, LOW);
      stepParallel(_n);
      digitalWrite(MOT_EN, HIGH);
    }
    
    digitalWrite(HOLD, LOW);
    
    while(1){
      delay(1000);
    }

  #else
    // ORIENTATION MEASUREMENT
      bno_0.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      qx_0 = orientationData.orientation.x;
      qy_0 = orientationData.orientation.y;
      qz_0 = orientationData.orientation.z;

      bno_1.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      qx_1 = orientationData.orientation.x - qx_1_ini;
      qy_1 = orientationData.orientation.y - qy_1_ini;
      qz_1 = orientationData.orientation.z - qz_1_ini;

    // HELIOS MEASUREMENT
      heliosSensor.setInputMultiplexer(ADS122C04_MUX_AIN0_AIN2);
      h0 = heliosSensor.readRawVoltage() / 4096000.0;

      heliosSensor.setInputMultiplexer(ADS122C04_MUX_AIN1_AIN3);
      h1 = heliosSensor.readRawVoltage() / 4096000.0;

    // PRINT DATA
    if(digitalRead(A1))
    {
      Serial.print(qx_1, 6);  Serial.print(",");
      Serial.print(qy_1, 6);    Serial.print(",");
      Serial.print(qz_1, 6);    Serial.print(",");
      Serial.print(h0, 6);     Serial.print(",");
      Serial.println(h1, 6);
    }
  #endif
}