// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#include "LSM6DS3.h"

void set_acc( void )
{
  Wire.beginTransmission(0x6B);
  Wire.write(LSM6DS3_ACC_GYRO_CTRL1_XL);
  Wire.write((LSM6DS3_ACC_GYRO_FS_XL_16g) | (LSM6DS3_ACC_GYRO_ODR_XL_104Hz) | (LSM6DS3_ACC_GYRO_FS_125_ENABLED));
  Wire.endTransmission();
}

void set_gyro( void )
{
  Wire.beginTransmission(0x6B);
  Wire.write(LSM6DS3_ACC_GYRO_CTRL2_G);
  Wire.write(LSM6DS3_ACC_GYRO_FS_125_ENABLED | LSM6DS3_ACC_GYRO_FS_G_2000dps | LSM6DS3_ACC_GYRO_ODR_G_104Hz);
  Wire.endTransmission();

}

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  Serial.println("Test Phrase");
  set_acc();
  set_gyro();
}

void loop()
{
  Wire.beginTransmission(0x6B);
  Wire.write(LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
  Wire.endTransmission();
  Wire.requestFrom(0x6B, 1);    // request 6 bytes from slave device #2

  while (Wire.available())   // slave may send less than requested
  {
    char c = Wire.read(); // receive a byte as character
    char c_high = (c & 0xF0) >> 4;
    char c_low = (c & 0x0F);
    Serial.print(c_high, HEX);         // print the character
    Serial.print(c_low, HEX);
    Serial.print(" ");
  }
  
  Serial.println("");
  Wire.beginTransmission(0x6B);
  Wire.write(LSM6DS3_ACC_GYRO_OUTX_L_G);
  Wire.endTransmission();
  
  Wire.requestFrom(0x6B, 12);    // request 6 bytes from slave device #2

  byte buf_ptr = 0;
  byte buf[12];
  while (Wire.available())   // slave may send less than requested
  {
    char c = Wire.read(); // receive a byte as character
    buf[buf_ptr] = c;
    char c_high = (c & 0xF0) >> 4;
    char c_low = (c & 0x0F);
    Serial.print(c_high, HEX);         // print the character
    Serial.print(c_low, HEX);
    Serial.print(" ");
    buf_ptr++;
  }
  //format numbers and output
  int temp;

  temp = buf[0] | (buf[1] << 8);
  Serial.print("\nGyro: ");
  Serial.print(temp);
  temp = buf[2] | (buf[3] << 8);
  Serial.print(" ");
  Serial.print(temp);
  temp = buf[4] | (buf[5] << 8);
  Serial.print(" ");
  Serial.print(temp);

  temp = buf[6] | (buf[7] << 8);
  Serial.print("\nAcc: ");
  Serial.print(temp);
  temp = buf[8] | (buf[9] << 8);
  Serial.print(" ");
  Serial.print(temp);
  temp = buf[10] | (buf[11] << 8);
  Serial.print(" ");
  Serial.print(temp);
  
  Serial.print("\n\n");
  delay(200);
  
}
