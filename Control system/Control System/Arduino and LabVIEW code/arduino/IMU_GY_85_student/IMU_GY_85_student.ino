/*
GY-85 9DoF IMU Arduino Code
Written by Russell SA Brinkworth 2014
Based on code found in FreeIMU Arduino examples by Fabio Varesano, Filipe Vieira and TJS
Using ADXL345 (accelerometer), ITG3200 (gyroscope) and HMC5883 (magnotometer)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

See <http://www.gnu.org/licenses/> for a copy of the GNU General Public License.
*/

#include <Wire.h> // I2C library
#include "ADXL345.h" // acc library
#include <HMC58X3.h> // mag library
#include <ITG3200.h> // gyro library

ADXL345 Accel;
HMC58X3 magn;
ITG3200 gyro = ITG3200();

float IMU_data[9]; // setup array for IMU data. 3xacceleration, 3x magnotometer and 3x gyro

float Acc_data[3]; // accelerometer needs a vector to put data into
float Acc_mag = 1; // normalisation factor for accelerometer

float Mag_offset[3] = {49.9399, -25.8, -47.7727}; // magnetometer offset callibration values. These will be different for every IMU. You need to callibrate them
float Mag_scale[3] = {811.523, 741.384, 763.838}; // magnetometer scale callibration. These will be different for every IMU. You need to callibrate them

float Gyro_data[3]; // gyro needs a vector to put data into
float Gyro_past[3] = {0, 0, 0}; // nominal gyro offset values for all 3 axes to remove drift. Must be very close to real value to start with but will be updated by the adaptive filter
float Gyro_scale = 54; // gyroscope scale value. Convert to rad/s
#define gyro_limit 5 // gyro movement limit before stop including value in drift correction calculation (adaptive filter)
#define TCgyro 100.0 // time constant for gyro offset calculation (x, y, z)

int loop_time = 100; // time to run the loop (ms)
int loop_delay = 0; // time to sleep in order for the loop time to be maintained
int loop_previous = millis(); // time of previous loop

void setup(void)
{
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Initialising accelerometer...");
  
  Accel.init(ADXL345_ADDR_ALT_LOW); // intialise accelerometer
  Accel.set_bw(ADXL345_BW_12); // set byte width
  
  Serial.println("Calibrating magnetometer...");
  magn.init(true); // no delay needed as we have already a delay(5) in HMC58X3::init()
  magn.calibrate(1, 10); // Calibrate HMC using self test, not recommended to change the gain after calibration. (Gain, number of sample). Use gain 1=default, valid 0-7, 7 not recommended.
  magn.setMode(0); // Single mode conversion was used in calibration, now set continuous mode
  
  Serial.println("Calibrating gyro...");
  gyro.reset();
  gyro.init(ITG3200_ADDR_AD0_LOW);
  gyro.zeroCalibrate(250,2); // initial drift correction on gyro. (Total number of samples, sample delay)
  
  Serial.println("IMU initialised");
}

void loop()
{ 
  Accel.get_Gxyz(Acc_data); // Get accelerometer data
  
  // calculate accelerometer normalisation factor
  Acc_mag = sqrt(Acc_data[0]*Acc_data[0] + Acc_data[1]*Acc_data[1] + Acc_data[2]*Acc_data[2]);
  for (int i = 0; i <= 2; i++)
  {
    IMU_data[i] = Acc_data[i] / Acc_mag; // normalise accelerometer data and put into IMU vector
  }
  
  magn.getValues(&IMU_data[3],&IMU_data[4],&IMU_data[5]); // Get magnetometer values
  for (int i = 0; i <= 2; i++)
  {
    IMU_data[i+3] = (IMU_data[i+3] - Mag_offset[i]) / Mag_scale[i]; // remove magnetometer offsets and apply scale
  }
  
  gyro.readGyro(Gyro_data); // Get gyro data
  for (int i = 0; i <= 2; i++)
  {
    // drift calculation for gyro
    if (abs(Gyro_data[i]-Gyro_past[i]) < gyro_limit) // only update adaptive filter if no movement detected in axis
    {
      // Gyro_past[i] = Gyro_past[i];// filter the gyro measurement to estimate offset for drift removal. Insert low-pass filter here
      Gyro_past[i] = (1/TCgyro)*Gyro_data[i] + (1-1/TCgyro)*Gyro_past[i];
    }
    IMU_data[i+6] = loop_time * (Gyro_data[i] - Gyro_past[i]) / (1000 * Gyro_scale); // put gyro data into IMU vector with drift removed. Scale data to rad
  }
  
  Serial.print("S: "); // print start data code
  for (int i = 0; i <= 7; i++) // print data to serial monitor
  {
    Serial.print(IMU_data[i], 6); // prints data using 6 decimal places
    Serial.print(", "); // seperate data with comma and space
  }
  Serial.print(IMU_data[8], 6); // print last data point without comma
  Serial.println(" :E"); // print end data code
  
  loop_delay = 100 + loop_previous - millis(); // calculate delay required to keep requested loop rate
  loop_previous = millis(); // put current time into variable to remember for next loop
  if (loop_delay > 0)
  {
    delay(loop_delay); // only pause program if necessary. Takes into account processing time for main loop
  }
}
