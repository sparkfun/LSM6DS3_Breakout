#include "Sparkfun_LSM6DS3.h"
#include "Wire.h"

//Constructor
Sparkfun_LSM6DS3::Sparkfun_LSM6DS3( void ) {
  settings.gyroEnabled = 1;  //Can be 0 or 1
  settings.gyroScale = 1; //Can be 1, 2, 4, 6, 8 ..
  settings.gyroSampleRate = 1; //Can be...
  settings.accelEnabled = 1;
  settings.accelScale = 1;
  settings.accelSampleRate = 1;
  settings.magEnabled = 0;
  settings.magScale = 1;
  settings.magSampleRate = 1;
  settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE
  settings.commAddress = 0x6B; //Ignored for SPI_MODE
  settings.tempEnabled = 0;

  xAccel = 0;
  yAccel = 0;
  zAccel = 0;
  xGyro = 0;
  yGyro = 0;
  zGyro = 0;
  xMag = 0;
  yMag = 0;
  zMag = 0;
  celsiusTemp = 0;
  fahrenheitTemp = 0;

}

//Call to utilize the settings
uint8_t Sparkfun_LSM6DS3::begin( void ) {
  //Check the settings structure values to determine how to setup the device
  
  Wire.begin();
  
  //Setup the accelerometer
  if ( settings.accelEnabled == 1) {
    Wire.beginTransmission(settings.commAddress);
    Wire.write(LSM6DS3_ACC_GYRO_CTRL1_XL);
    Wire.write((LSM6DS3_ACC_GYRO_FS_XL_16g) | (LSM6DS3_ACC_GYRO_ODR_XL_104Hz) | (LSM6DS3_ACC_GYRO_FS_125_ENABLED));
    Wire.endTransmission();
  }
  
  //Setup the gyroscope
  if ( settings.gyroEnabled == 1) {
    Wire.beginTransmission(settings.commAddress);
    Wire.write(LSM6DS3_ACC_GYRO_CTRL2_G);
    Wire.write(LSM6DS3_ACC_GYRO_FS_125_ENABLED | LSM6DS3_ACC_GYRO_FS_G_2000dps | LSM6DS3_ACC_GYRO_ODR_G_104Hz);
    Wire.endTransmission();
  }
  
  //Setup the magnetometer
  if ( settings.magEnabled == 1) {
  }
  
  //Setup the internal temperature sensor
  if ( settings.tempEnabled == 1) {
  }
  
  //Return WHO AM I reg
  Wire.beginTransmission(settings.commAddress);
  Wire.write(LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
  Wire.endTransmission();
  Wire.requestFrom(settings.commAddress, 1);    // request 6 bytes from slave device #2
  uint8_t c;
  while (Wire.available())   // slave may send less than requested
  {
    c = Wire.read(); // receive a byte as character
  }
  
  return c;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
void Sparkfun_LSM6DS3::readAccel( void ) {
  uint8_t myBuffer[6];
  getRegion(myBuffer, LSM6DS3_ACC_GYRO_OUTX_L_XL, 6);  //Does memory transfer

  //Do the math to get from raw numbers to useful parts.  Save into class.
  xAccel = myBuffer[0] | (myBuffer[1] << 8);
  yAccel = myBuffer[2] | (myBuffer[3] << 8);
  zAccel = myBuffer[4] | (myBuffer[5] << 8);

}

int16_t Sparkfun_LSM6DS3::readAccel( uint8_t offset ) {
  uint8_t myBuffer[2];
  getRegion(myBuffer, (LSM6DS3_ACC_GYRO_OUTX_L_XL + (offset << 1)), 2);  //Does memory transfer

  //Do the math to get from raw numbers to useful int
  int16_t outputVariable = myBuffer[0] | (myBuffer[1] << 8);
 
  return outputVariable;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//

void Sparkfun_LSM6DS3::readGyro( void ) {
  uint8_t myBuffer[6];
  getRegion(myBuffer, LSM6DS3_ACC_GYRO_OUTX_L_G, 6);  //Does memory transfer

  //Do the math to get from raw numbers to useful parts.  Save into class.
  xAccel = myBuffer[0] | (myBuffer[1] << 8);
  yAccel = myBuffer[2] | (myBuffer[3] << 8);
  zAccel = myBuffer[4] | (myBuffer[5] << 8);

}

int16_t Sparkfun_LSM6DS3::readGyro( uint8_t offset ) {
  uint8_t myBuffer[2];
  getRegion(myBuffer, (LSM6DS3_ACC_GYRO_OUTX_L_G + (offset << 1)), 2);  //Does memory transfer

  //Do the math to get from raw numbers to useful int
  int16_t outputVariable = myBuffer[0] | (myBuffer[1] << 8);
 
  return outputVariable;
}

//****************************************************************************//
//
//  Mag section
//
//****************************************************************************//

void Sparkfun_LSM6DS3::readMag( void ) {
}

int16_t Sparkfun_LSM6DS3::readMag( uint8_t ) {
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//

int16_t Sparkfun_LSM6DS3::readTemp( void ) {
}

void Sparkfun_LSM6DS3::readAll( void ) {
  readAccel();
  readGyro();
  readMag();
  readTemp();
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void Sparkfun_LSM6DS3::getRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length) {

  //define pointer that will point to the external space
  //uint8_t * outputPointer;
  //outputPointer = &outputRegion;
  
  Wire.beginTransmission(settings.commAddress);
  Wire.write(offset);
  Wire.endTransmission();
  
  Wire.requestFrom(settings.commAddress, length);    // request 6 bytes from slave device #2

  while (Wire.available())   // slave may send less than requested
  {
    char c = Wire.read(); // receive a byte as character
    *outputPointer = c;
    outputPointer++;
  }

}
