#ifndef SENSOR_H
#define SENSOR_H

#include "stdint.h"

struct SensorSettings {
    public:
	uint8_t gyroEnabled;
    uint8_t gyroScale;
    uint8_t gyroSampleRate;
    uint8_t accelEnabled;
    uint8_t accelScale;
    uint8_t accelSampleRate;
    uint8_t magEnabled;
    uint8_t magScale;
    uint8_t magSampleRate;
    uint8_t commInterface;
    uint8_t commAddress;
    uint8_t tempEnabled;
};

#define I2C_MODE 0
#define SPI_MODE 1

#define X_DIR 0
#define Y_DIR 1
#define Z_DIR 2

class SensorPartNumber {

public:
	SensorSettings settings;

	int16_t xAccel;
	int16_t yAccel;
	int16_t zAccel;
	int16_t xGyro;
	int16_t yGyro;
	int16_t zGyro;
	int16_t xMag;
	int16_t yMag;
	int16_t zMag;
	int16_t celsiusTemp;
    int16_t fahrenheitTemp;

	SensorPartNumber( void );  //Constructor
	void begin();  //Call to utilize the settings
	void readAccel( void );
	int16_t readAccel( uint8_t );
	void readGyro( void );
	int16_t readGyro( uint8_t );
	void readMag( void );
	int16_t readMag( uint8_t );
	void readAll( void );
	int16_t readTemp( void );
};
#endif // SENSOR_H
