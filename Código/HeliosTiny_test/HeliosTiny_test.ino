#include "HeliosSection.h"
#include "cmd_list.h"

HeliosData sensorData;
int cmd;

void setup() {
  initComms();
  initSensor();
  delay(500);
}

void loop() {
  //sensorData = readSensors();  
    switch(cmd)
    {
      case CMD_DATA_REQ:
        break;

      case CMD_LED_OFF:
        digitalWrite(LED, LOW);
        break;

      case CMD_LED_ON:
        digitalWrite(LED, HIGH);
        break;

      default:
        break;
    }
}
