/**************************************************************************/
/*!
    @file     Adafruit_LIS3DH.cpp
    @author   K. Townsend / Limor Fried (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit LIS3DH Accel breakout board
    ----> https://www.adafruit.com/products/2809

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
	v7 - Adapted for repetier
	
	************** NOTE **************
	This library has been severely cut down and adapted for use in Repetier
	-B Parcels
*/
/**************************************************************************/

#include "Repetier.h"



/**************************************************************************/
/*!
    @brief  Instantiates a new LIS3DH class in I2C or SPI mode
*/
/**************************************************************************/
// I2C
Adafruit_LIS3DH::Adafruit_LIS3DH() {
}



/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool Adafruit_LIS3DH::begin(uint8_t i2caddr) {
  _i2caddr = i2caddr;
    
    // i2c
    //I2Cinterface->begin();  

  /* Check connection */
  uint8_t deviceid = readRegister8(LIS3DH_REG_WHOAMI);
  if (deviceid != 0x33)
  {
    /* No LIS3DH detected ... return false */
    //Serial.println(deviceid, HEX);
    return false;
  }

  // enable all axes, normal mode
  writeRegister8(LIS3DH_REG_CTRL1, 0x07);
  // 400Hz rate
  setDataRate(LIS3DH_DATARATE_400_HZ);

  // High res & BDU enabled
  writeRegister8(LIS3DH_REG_CTRL4, 0x88);

  // No interrupts
  writeRegister8(LIS3DH_REG_CTRL3, 0x00);
  
  // disable temp/adcs
  writeRegister8(LIS3DH_REG_TEMPCFG, 0x00);

  return true;
}

bool Adafruit_LIS3DH::IsConnected() {
	uint8_t deviceid = readRegister8(LIS3DH_REG_WHOAMI);
	if (deviceid != 0x33)
	{
		/* No LIS3DH detected ... return false */
		//Serial.println(deviceid, HEX);
		return false;
	}
	return true;
}

/**************************************************************************/
/*!
    @brief  Set INT to output for single or double click
*/
/**************************************************************************/

void Adafruit_LIS3DH::setClick(uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow) {
  if (!c) {
    //disable int
    writeRegister8(LIS3DH_REG_CTRL3, 0);
    writeRegister8(LIS3DH_REG_CLICKCFG, 0);
    return;
  }
  // else...

  writeRegister8(LIS3DH_REG_CTRL3, 0x80); // turn on int1 click  

  if (c == 1)
    writeRegister8(LIS3DH_REG_CLICKCFG, 0x15); // turn on all axes & singletap
  if (c == 2)
    writeRegister8(LIS3DH_REG_CLICKCFG, 0x2A); // turn on all axes & doubletap


  writeRegister8(LIS3DH_REG_CLICKTHS, clickthresh | 0x80); // arbitrary threshold - NOTE - the interrupt latch bit is bit 8 here, so we are setting it to latch!
  writeRegister8(LIS3DH_REG_TIMELIMIT, timelimit); // arbitrary
  writeRegister8(LIS3DH_REG_TIMELATENCY, timelatency); // arbitrary
  writeRegister8(LIS3DH_REG_TIMEWINDOW, timewindow); // arbitrary
}

uint8_t Adafruit_LIS3DH::getClick(void) {
  return readRegister8(LIS3DH_REG_CLICKSRC);
}


/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void Adafruit_LIS3DH::setRange(lis3dh_range_t range)
{
  uint8_t r = readRegister8(LIS3DH_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  writeRegister8(LIS3DH_REG_CTRL4, r);
}


/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
void Adafruit_LIS3DH::setDataRate(lis3dh_dataRate_t dataRate)
{
  uint8_t ctl1 = readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  writeRegister8(LIS3DH_REG_CTRL1, ctl1);
}



/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void Adafruit_LIS3DH::writeRegister8(uint8_t reg, uint8_t value) {  
	
	//HAL::delayMilliseconds(100);
	HAL::i2cStartWait(_i2caddr << 1 | I2C_WRITE);
	HAL::i2cWrite((uint8_t)reg);
	HAL::i2cWrite((uint8_t)value);
	HAL::i2cStop();  
}


//// I2C Support
//static void i2cSetClockspeed(uint32_t clockSpeedHz);
//static void i2cInit(unsigned long clockSpeedHz);
//static void i2cStartWait(unsigned char address);
//static uint8_t i2cStart(unsigned char address);
//static void i2cStartAddr(unsigned char address, unsigned int pos);
//static void i2cStop(void);
//static void i2cStartBit(void);
//static void i2cCompleted(void);
//static void i2cTxFinished(void);
//static void i2cWrite(uint8_t data);
//static uint8_t i2cReadAck(void);
//static uint8_t i2cReadNak(void);

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t Adafruit_LIS3DH::readRegister8(uint8_t reg) {	
	uint8_t value;
	
	//HAL::delayMilliseconds(500);
	HAL::i2cStartWait(_i2caddr << 1 | I2C_WRITE);
	HAL::i2cWrite((uint8_t)reg); 
	HAL::i2cStop();
	HAL::i2cStart(_i2caddr << 1 | I2C_READ);
	value = HAL::i2cReadNak();
	HAL::i2cStop();

	return value;
}
