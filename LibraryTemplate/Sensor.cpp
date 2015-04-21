#include "Sensor.h"

//Constructor
SensorPartNumber::SensorPartNumber( void ){
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
	settings.commAddress = 0x6A; //Ignored for SPI_MODE
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
void SensorPartNumber::begin( void ){
}
void SensorPartNumber::readAccel( void ){
}
int16_t SensorPartNumber::readAccel( uint8_t ){
}
void SensorPartNumber::readGyro( void ){
}
int16_t SensorPartNumber::readGyro( uint8_t ){
}
void SensorPartNumber::readMag( void ){
}
int16_t SensorPartNumber::readMag( uint8_t ){
}
void SensorPartNumber::readAll( void ){
}
int16_t SensorPartNumber::readTemp( void ){
}
