#include "HeliosSection.h"
#include <ADS1115_WE.h>

ADS1115_WE adc = ADS1115_WE(I2C_ADDR);

void initSensor()
{
  pinMode(LED, OUTPUT);
  adc.init();

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range
  adc.setMeasureMode(ADS1115_SINGLE); //comment line/change parameter to change mode
}

int16_t readChannel(ADS1115_MUX channel)
{
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();

  while(adc.isBusy()){}  
  return adc.getRawResult();
}

HeliosData readSensors()
{
  HeliosData data;
  digitalWrite(LED, HIGH);
  delay(LED_ON_TIME);

  data.f0 = readChannel(ADS1115_COMP_0_GND);
  data.f1 = readChannel(ADS1115_COMP_1_GND);
  data.f2 = readChannel(ADS1115_COMP_2_GND);
  data.f3 = readChannel(ADS1115_COMP_3_GND);

  digitalWrite(LED, LOW);
  return data;
}