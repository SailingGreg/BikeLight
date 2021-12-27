#include "BMA250.h"
#include <inttypes.h>
#include "Arduino.h"
#include <Wire.h>

BMA250::BMA250()
{
}

int BMA250::begin(uint8_t range, uint8_t bw)
{
  //Detect address
  I2Caddress = BMA250_I2CADDR;
  Wire.beginTransmission(I2Caddress);
  if (Wire.endTransmission()) {
    I2Caddress++;
    Wire.beginTransmission(I2Caddress);
    if (Wire.endTransmission()) {
      I2Caddress = 0;
      return -1;
    }
  }
  //Setup the range measurement setting
  Wire.beginTransmission(I2Caddress);
  Wire.write(0x0F); // 0x0f is the range register
  Wire.write(range);
  Wire.endTransmission();
  //Setup the bandwidth
  Wire.beginTransmission(I2Caddress);
  Wire.write(0x10); // 0x10 is the bandwidth register
  Wire.write(bw);
  Wire.endTransmission();

  /* 
      Accel_write(0x21,0x09);//Interrupt mode temporary 500us
      Accel_write(0x16,0x07);//Enable slope interrupts
      Accel_write(0x19,0x04);//Map slope interrupt to INT1 pin
      Accel_write(0x27,0x3);//Use 4 samples for slope interrupt
      Accel_write(0x28,0x14);//Set slope threshhold to 0x14
   */

   // set the interrupt mode - this is temporary 500 usec
   // note we don't not latched the interrupt
   Wire.beginTransmission(I2Caddress);
   Wire.write(0x21); // interrupt mode - see page 23 of the spec
   Wire.write(0x09); 
   Wire.endTransmission();
   // enable slope interrupts - used for 'any motion' detection
   Wire.beginTransmission(I2Caddress);
   Wire.write(0x16);
   Wire.write(0x07);
   Wire.endTransmission();
   // Map slope interrupt to pin 1
   Wire.beginTransmission(I2Caddress);
   Wire.write(0x19);
   Wire.write(0x04);
   Wire.endTransmission();
   // Use 4 samples for the slope interrupt - threshold duration
   Wire.beginTransmission(I2Caddress);
   Wire.write(0x27);
   Wire.write(0x02); // reduce to 3 from 4
   Wire.endTransmission();
   // And set slope threshold
   Wire.beginTransmission(I2Caddress);
   Wire.write(0x28); // slope threshold 
   Wire.write(0x14); // note this is the default value - see page 47 of the spec
   Wire.endTransmission();

  return 0;
}

void BMA250::read()
{
  //Set register index
  Wire.beginTransmission(I2Caddress);
  Wire.write(0x02); // the register to read - start at 0x02 to 0x09
  Wire.endTransmission();
  //Request seven data bytes
  Wire.requestFrom(I2Caddress, 7); // number of resgisters to be read
  //Receive acceleration measurements as 16 bit integers
  X = (int16_t)Wire.read();
  X |= (int16_t)Wire.read() << 8;
  Y = (int16_t)Wire.read();
  Y |= (int16_t)Wire.read() << 8;
  Z = (int16_t)Wire.read();
  Z |= (int16_t)Wire.read() << 8;
  //Only use the 10 significant bits
  X >>= 6; Y >>= 6; Z >>= 6;
  //Receive temperature measurement
  rawTemp = Wire.read();
  tempC = rawTemp/2 + 23;
}

//uint8_t interruptStatus;

void BMA250::readInter()
{  
  // should return '4' for a slope interrupt
  Wire.beginTransmission(I2Caddress);
  Wire.write(0x09); // interrupt status register - see page 39 of the spec
  Wire.endTransmission();
  Wire.requestFrom(I2Caddress, 1);
  
  interruptStatus = Wire.read();
}
