#ifndef __HELIOS_SECTION__
  #define __HELIOS_SECTION__
  #include <Arduino.h>

  /* ATTINY25/45/84/85 PINOUT
               _________
              |         |
  PB5/!RESET [|1       8|] VCC
     PB3/LED [|2       7|] PB2/SCK/SCL
  PB4/SW_SCL [|3       6|] PB1/MISO/SW_SDA
         GND [|4       5|] PB0/MOSI/SDA
              |_________|
  */

  const uint8_t SW_SDA = PB1;
  const uint8_t SW_SCL = PB4;
  const uint8_t LED = PB3;

  // TIMING
  const int LED_ON_TIME = 5; //ms
  const int I2C_ASYNC_DELAY = 5; //us

  // DATA
  struct HeliosData
  {
    int16_t f0, f1, f2, f3;
  };

  // COMMS
  const uint8_t I2C_ADDR = 'F';
  
  void initComms();
  int listenCMD();

  // SENSING
  void initSensor();
  HeliosData readSensors();

#endif