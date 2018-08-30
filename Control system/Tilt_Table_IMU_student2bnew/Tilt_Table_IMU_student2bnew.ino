/*
GY-85 9DoF IMU Arduino Code
Written by Russell SA Brinkworth 2014
Based on code found in FreeIMU Arduino examples by Fabio Varesano, Filipe Vieira and TJS
Using ADXL345 (accelerometer), ITG3200 (gyroscope) and HMC5883 (magnotometer)
Modified to only use accelerometer and gyroscope for use in the Tilt Table Practical

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

See <http://www.gnu.org/licenses/> for a copy of the GNU General Public License.
*/

/*********************************************************************************
 **
 ** Libraries to include
 **
 ********************************************************************************/ 
#include <Wire.h> // I2C library
#include "ADXL345.h" // acc library
#include <ITG3200.h> // gyro library
#include <math.h> // maths library
#include <Servo.h>

/* -------- Servo specific section -------- */
#define innerPlatePin 11        //-- pin number for inner servo
#define outterPlatePin 10       //-- pin number for outer servo
Servo innerPlate, outerPlate;   //-- Create servo objects for outer and inner plates  
float Plate_ref[2] = {0.0, 0.0};      //-- Variable to store recieved servo platform position        
float sensitivity = 100.0;
int Offset[2] = {1300, 1910};   //-- Increasing 1st value moves the inner plate away from the outer servo direction. Increasing 2nd value moves outer plate towards the inner servo direction
int32_t servo[2];               // variable for the servo values
int Servo_limits[2] = {2400, 550}; //-- maximum and minimum value that can be written to the servo. Can cause errors if these limits are exceeded

/* -------- Communications specific section -------- */
int incomingByte = 0;       // for incoming serial data from host
#define message_length 3    // length of message to be sent from the host
#define update_rate 5       // number of loops to go before reporting values to host
int Setpoint[2];            // setpoints from host
float control_scale = 0.1;    // conversion factor from integer to PWM value
float Plate_reference[2];   // desited reference setpoints from host (deg)


/* -------- IMU specific section -------- */
ADXL345 Accel;
ITG3200 gyro = ITG3200();

float IMU_data[6]; // setup array for IMU data. 3xacceleration and 3x gyro

float Acc_data[3]; // accelerometer needs a vector to put data into
float Acc_scale = 1; // normalisation factor for accelerometer
float Acc_offset[2] = {-4.6496,-2.11254}; // mounting offset for accelerometer. Modify this value if the table is not level on startup

float Gyro_data[3]; // gyro needs a vector to put data into
float Gyro_past[3] = {0, 0, 0}; // nominal gyro offset values for all 3 axes to remove drift. Must be very close to real value to start with but will be updated by the adaptive filter
float Gyro_scale = 1; // gyroscope scale value. Need to convert to degrees
#define gyro_limit 0.5 // gyro movement limit before stop including value in drift correction calculation (adaptive filter)
#define TCgyro 100.0 // time constant for gyro offset calculation (x, y, z)

float Angle_data[4] = {-5000, -5000, -5000, -5000}; // setup array for plate angle estimates. Set initial value to crazy level so easy to identify first run

/* -------- Timing specific section -------- */
unsigned long loop_time = micros();   // loop time place holder for loop delay calculation
#define loop_delay 10000 	      // loop delay in us
int loop_number = 1;                  // loop number tracker for reporting values to host

/* -------- Communications specific section -------- */
#define update_rate 5       // number of loops to go before reporting values to host

/*********************************************************************************
 **  setup()
 **
 **  Initialize the Arduino and setup serial communication.
 **
 *********************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Wire.begin();
  
  //-- Servo hardware init
  innerPlate.attach(innerPlatePin);
  outerPlate.attach(outterPlatePin);

  //-- Set the servos to zero pint;
  innerPlate.writeMicroseconds(Offset[0]);
  outerPlate.writeMicroseconds(Offset[1]);
  
  Serial.println("Initialising accelerometer...");
  
  Accel.init(ADXL345_ADDR_ALT_LOW); // intialise accelerometer
  Accel.set_bw(ADXL345_BW_12); // set byte width
  
  Serial.println("Calibrating gyro...");
  gyro.reset();
  gyro.init(ITG3200_ADDR_AD0_LOW);
  gyro.zeroCalibrate(250,2); // initial drift correction on gyro. (Total number of samples, sample delay)
  
  Serial.println("IMU initialised");
}

/*********************************************************************************
 **
 **  Main Loop
 **
 *********************************************************************************/
void loop()
{ 
  Loop_Timing();
  Read_Serial_Port();
  Update_Servos();
  Get_IMU_Data();
  IMU_to_Angle();
 
  
  
  if (loop_number >= update_rate) // only report values back to host every 'update_rate' loop
  {
    Serial.print("S: "); // print start data code
    for (int i = 0; i <= 2; i++) // print data to serial monitor
    {
      Serial.print(Angle_data[i], 6); // prints data using 6 decimal places
      Serial.print(" "); // seperate data with space
    }
    Serial.print(Angle_data[3], 6); // print last data point without space
    Serial.println(" :E"); // print end data code
    
    loop_number = 1;
  }
  else
  {
    loop_number = loop_number + 1;
  }
}

/*********************************************************************************
 **
 **  Subroutines
 **
 *********************************************************************************/
 
void Loop_Timing()
{
  if (micros() > loop_time)   // check overflow of micros() has not happened
  {
    while (micros()-loop_time < loop_delay - 150 && micros() > loop_time) // delay if needed but keep checking in case of interupts. Ensure no overflow in timer
    {
      delayMicroseconds(100);
    }
    if (micros()-loop_time < loop_delay - 5 && micros() > loop_time) // delay for the last bit if needed. Ensure no overflow in timer
    {
      delayMicroseconds(loop_delay-(micros()-loop_time)-5); // stay slightly ahead of required speed
    }
  }
  loop_time = micros(); // save current time for next loop
}

void Get_IMU_Data()
{
  Accel.get_Gxyz(Acc_data); // Get accelerometer data
  Acc_scale = sqrt(Acc_data[0]*Acc_data[0] + Acc_data[1]*Acc_data[1] + Acc_data[2]*Acc_data[2]); // calculate normalisation factor. Put normalisation formula in here
  for (int i = 0; i <= 2; i++)
  {
    IMU_data[i] = Acc_data[i] / Acc_scale; // normalise accelerometer data and put into IMU vector
  }
  
  gyro.readGyro(Gyro_data); // Get gyro data
  for (int i = 0; i <= 2; i++)
  {
    // drift calculation for gyro
    if (abs(Gyro_data[i]) < gyro_limit) // only update adaptive filter if no movement detected in axis
    {
      Gyro_past[i] = (1/TCgyro)*Gyro_data[i] + (1-1/TCgyro)*Gyro_past[i]; // filter the gyro measurement to estimate offset for drift removal. Put low-pass filter formula in here
    }
    IMU_data[i+3] =  (Gyro_data[i] - Gyro_past[i]) / Gyro_scale; // put gyro data into IMU vector with drift removed. Scale data to degrees
  }
}

void IMU_to_Angle()
  {
    Angle_data[0] = atan2(-Acc_data[0], -Acc_data[2]); // convert accelerometer data into an angle
    Angle_data[1] = atan2(-Acc_data[1], -Acc_data[2]);
    Angle_data[0] = Angle_data[0]*(180/3.142);
    Angle_data[1] = Angle_data[1]*(180/3.142);
    Angle_data[0] = Angle_data[0] - Acc_offset[0]; // remove misalignment from accelerometer angular estimation
    Angle_data[1] = Angle_data[1] - Acc_offset[1];
    
    if (Angle_data[2] == -5000) // determine if first run as gyroscope values are only a relative, not absolute, measurement so need an initial reference
    {
    Angle_data[2] = Angle_data[0]; // if first run use accelerometer values, not gyro
    Angle_data[3] = Angle_data[1];
    }
    else
    {
    Angle_data[2] = Angle_data[2] - IMU_data[4]; // if not first run then change angular estimate based on new gyro values
    Angle_data[3] = Angle_data[3] + IMU_data[3]; // note that the inner plate reference is the -ve of the 2nd gyro value and the outer plate reference is the +ve of the 1st gyro value
    }
  }
  
  void Read_Serial_Port()
 {
   while (Serial.available() > message_length) // check if there is a significant buffer backlog
  {
    incomingByte = Serial.read();  // empty incomming buffer of old messages
  }
  if (Serial.available() >= message_length) // if serial data has the required number of bytes avaiable for the message
  {
    incomingByte = Serial.read();  // read from serial buffer
    if (incomingByte == 128) // see if the byte is the control byte for the start of a message
    {
      Setpoint[0] = Serial.read();  // read inner sepoint value
      Setpoint[1] = Serial.read();  // read outer setpoint value
      
      Plate_ref[0] = convertByte(Setpoint[0]); // convert to deg
      Plate_ref[1] = convertByte(Setpoint[1]);
    }
  }
 }
 
 void Update_Servos()
{
  servo[0] = Offset[0] + int(sensitivity * Plate_ref[0]);   //-- No control system code. Just pass value out to motors
  servo[1] = Offset[1] + int(sensitivity * Plate_ref[1]);
  
  // Check servo values are within limits
  for (int i = 0; i <=1; i++)
  {
    if (servo[i] > Servo_limits[0])
    {
      servo[i] = Servo_limits[0]; // do not let servo value be larger than defined limit
    }
    else if (servo[i] < Servo_limits[1])
    {
      servo[i] = Servo_limits[1]; // do not let servo value be smaller than defined limit
    }
  }
  
  // Write updated value to Servo
  innerPlate.writeMicroseconds(servo[0]);
  outerPlate.writeMicroseconds(servo[1]);
}

float convertByte(int value) // converts a byte into an angle
{
  float result;
  if (value > 127) // convert from unsigned to signed integer
  {
    result = value - 256;
  }
  else
  {
    result = value;
  }
  result = result * control_scale;
  
  return result;
}
