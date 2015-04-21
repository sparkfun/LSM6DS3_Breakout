#include <Wire.h>
#include "Sparkfun_LSM6DS3.h"
#include "stdint.h"

//Don't pass things to the constructor
Sparkfun_LSM6DS3 mySensor;

void setup( void ) {
  Serial.begin(9600);  // start serial for output
  Serial.println("Test Phrase");
  //Over-ride default settings if desired
  mySensor.settings.gyroEnabled = 1;  //Can be 0 or 1
  //mySensor.settings.gyroScale = 1; //Can be 1, 2, 4, 6, 8 ..
  //mySensor.settings.gyroSampleRate = 1; //Can be...
  mySensor.settings.accelEnabled = 1;
  //mySensor.settings.accelScale = 1;
  //mySensor.settings.accelSampleRate = 1;
  //mySensor.settings.magEnabled = 1;
  //mySensor.settings.magScale = 1;
  //mySensor.settings.magSampleRate = 1;
  //mySensor.settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE
  mySensor.settings.commAddress = 0x6B; //Ignored for SPI_MODE
  mySensor.settings.tempEnabled = 1;

  uint8_t c = mySensor.begin();  //Can be called again to load new settings
  Serial.print("begin() ran.  Returns WHO_AM_I of: 0x");
  Serial.println(c, HEX);

}

void loop() {
  //To get a single parameter.  These return
  int16_t temp;
  temp = mySensor.readAccel(X_DIR);  //Raw
  Serial.println(temp);
  temp = mySensor.readAccel(Y_DIR);  //Raw
  Serial.println(temp);
  temp = mySensor.readAccel(Z_DIR);  //Raw
  Serial.println(temp);

  temp = mySensor.readGyro(X_DIR);  //Raw
  Serial.println(temp);

  temp = mySensor.readMag(X_DIR);  //Raw
  Serial.println(temp);

  temp = mySensor.readTemp();  //Raw
  Serial.println(temp);

  //To get all data, read into "the internal structure"
  mySensor.readAll();

  //Or, only update a single sensor's XYZ data
  mySensor.readAccel();

  //Now we can use
  Serial.println(mySensor.xAccel);
  Serial.println(mySensor.celsiusTemp);
  Serial.println(mySensor.fahrenheitTemp);
  Serial.println(mySensor.zGyro);

  //To stop it all
  //mySensor.stop();
    while(1);
}

