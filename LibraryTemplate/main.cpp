#include "stdint.h"
#include "Sensor.h"

//Don't pass things to the constructor
SensorPartNumber mySensor;

void setup( void ) {
    mySensor.zMag = 1;
  //Over-ride default settings if desired
  mySensor.settings.gyroEnabled = 1;  //Can be 0 or 1
  mySensor.settings.gyroScale = 1; //Can be 1, 2, 4, 6, 8 ..
  mySensor.settings.gyroSampleRate = 1; //Can be...
  mySensor.settings.accelEnabled = 1;
  mySensor.settings.accelScale = 1;
  mySensor.settings.accelSampleRate = 1;
  mySensor.settings.magEnabled = 1;
  mySensor.settings.magScale = 1;
  mySensor.settings.magSampleRate = 1;
  mySensor.settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE
  mySensor.settings.commAddress = 0x6A; //Ignored for SPI_MODE
  mySensor.settings.tempEnabled = 1;

  mySensor.begin();  //Can be called again to load new settings

}

int main() {  //void loop() {
  //To get a single parameter.  These return
  int16_t temp;
  temp = mySensor.readAccel(X_DIR);  //Raw
  temp = mySensor.readAccel(Y_DIR);  //Raw
  temp = mySensor.readAccel(Z_DIR);  //Raw

  temp = mySensor.readGyro(X_DIR);  //Raw

  temp = mySensor.readMag(X_DIR);  //Raw

  temp = mySensor.readTemp();  //Raw

  //To get all data, read into "the internal structure"
  mySensor.readAll();

  //Or, only update a single sensor's XYZ data
  mySensor.readAccel();

  //Now we can use
  temp = mySensor.xAccel;
  temp = mySensor.celsiusTemp;
  temp = mySensor.fahrenheitTemp;
  temp = mySensor.zGyro;


  //To stop it all
  //mySensor.stop();
    while(1);
}
