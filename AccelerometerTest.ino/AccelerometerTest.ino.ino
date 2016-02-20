
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>



// Use the LSM9DS1 class to create an object. ch.
LSM9DS1 imu;



double accel_ang_x;
double accel_ang_y;

double gyro_ang_x = 0;
double gyro_ang_y;

double raw_Accel_x;
double raw_Accel_y;
double raw_Accel_z;

double raw_Gyro_x;
double raw_Gyro_y;
double raw_Gyro_z;

//Timing Variables
long unsigned int start;
long unsigned int end_t;
long unsigned int delta_t = 0;

void setup() 
{
  
  
  Serial.begin(115200);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:

  imu.settings.device.commInterface = IMU_MODE_I2C; //Sets mag to I2C
  imu.settings.device.mAddress = 0x1E;
  imu.settings.device.agAddress = 0x6B; // Set ag address to 0x6B

  //Ensures There is a connection to the IMU
  if (!imu.begin()){
    Serial.println("Failed to Connect to IMU");

    //Loops Here Forever
    while(1){};
    }
  
}

void loop()
{   
  
   start = micros();
   imu.readAccel(); //Updates the acceleromoter data
   imu.readGyro();  //Updates the gyro data

   //Gets raw Acceleration Data
   raw_Accel_x = imu.calcAccel(imu.ax);
   raw_Accel_y = imu.calcAccel(imu.ay);
   raw_Accel_z = imu.calcAccel(imu.az);

   //Gets raw Gyro Data
   raw_Gyro_x = imu.calcGyro(imu.ax);
   raw_Gyro_y = imu.calcGyro(imu.az);
   raw_Gyro_z = imu.calcGyro(imu.az);
   

   accel_ang_x = atan(raw_Accel_x/sqrt((raw_Accel_y*raw_Accel_y)+(raw_Accel_z*raw_Accel_z)));
   accel_ang_y = atan(raw_Accel_y/sqrt((raw_Accel_x*raw_Accel_x)+(raw_Accel_z*raw_Accel_z)));

   gyro_ang_x = gyro_ang_x + (raw_Gyro_x*delta_t*0.000001);
   
  
   Serial.println(accel_ang_x);
   delay(50);
   
   
}



