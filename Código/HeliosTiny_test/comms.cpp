#include "HeliosSection.h"
#include <AsyncDelay.h>
#include <SoftWire.h>
#include <TinyWireM.h>

SoftWire swI2C(SW_SDA, SW_SCL);
AsyncDelay samplingInterval;
extern int cmd;

void initComms()
{	
  swI2C.setDelay_us(I2C_ASYNC_DELAY);
	swI2C.begin(I2C_ADDR);
  swI2C.onReceive(listenCMD);

  TinyWireM.begin(0x24);
}

int listenCMD(int howMany)
{
  if (howMany > 0) { // if there is data available
    cmd = swI2C.read(); // read the first byte
    swI2C.write(cmd); // echo back the same byte
  }
}