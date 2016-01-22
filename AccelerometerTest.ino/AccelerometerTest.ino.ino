
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>


// Use the LSM9DS1 class to create an object. ch.
LSM9DS1 imu;


void setup() 
{
  
  Serial.begin(115200);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:

  imu.settings.device.commInterface = IMU_MODE_I2C; //Sets mag to I2C
  imu.settings.device.mAddress = 0x1E;
  imu.settings.device.agAddress = 0x6B; // Set ag address to 0x6B
}

void loop()
{
  if (!imu.begin()){
    Serial.println("Failed to Connect to IMU");

    //Loops Here Forever
    while(1){};
    }

    imu.readAccel(); //Updates the acceleromoter data
    Serial.println(imu.calcAccel(imu.ax)); //Prints x-axis accceleration
  
}


