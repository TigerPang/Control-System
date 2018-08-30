/*********************************************************************************
 **
 **  Tilt Table Code for interface with LabVIEW
 **
 **  Modified for direct control of the servo motors only
 **
 **  Written By:    Russell SA Brinkworth - UniSA
 **  Written On:    March 2014
 **  Last Updated:  
 **
 *********************************************************************************/


/*********************************************************************************
 **
 ** Includes
 **
 ********************************************************************************/ 
// Libraries to include
#include <Servo.h> // servo library

/*********************************************************************************
 **
 ** Variables and Globals
 **
 ********************************************************************************/ 
/* -------- Servo specific section -------- */
#define innerPlatePin 11        //-- pin number for inner servo
#define outterPlatePin 10       //-- pin number for outer servo
Servo innerPlate, outerPlate;   //-- Create servo objects for outer and inner plates  
float Plate_ref[2] = {0.0, 0.0};      //-- Variable to store recieved servo platform position        
float sensitivity = 100.0;
int Offset[2] = {1500, 1500};   //-- Increasing 1st value moves the inner plate away from the outer servo direction. Increasing 2nd value moves outer plate towards the inner servo direction
int32_t servo[2];               // variable for the servo values
int Servo_limits[2] = {2400, 550}; //-- maximum and minimum value that can be written to the servo. Can cause errors if these limits are exceeded

/* -------- Communications specific section -------- */
int incomingByte = 0;       // for incoming serial data from host
#define message_length 3    // length of message to be sent from the host
#define update_rate 5       // number of loops to go before reporting values to host
int Setpoint[2];            // setpoints from host
float control_scale = 0.1;    // conversion factor from integer to PWM value
float Plate_reference[2];   // desited reference setpoints from host (deg)

/* -------- Timing specific section -------- */
unsigned long loop_time = micros();   // loop time place holder for loop delay calculation
#define loop_delay 50000 	      // loop delay in us
int loop_number = 1;                  // loop number tracker for reporting values to host

/*********************************************************************************
 **  setup()
 **
 **  Initialize the Arduino and setup serial communication.
 **
 *********************************************************************************/
void setup()
{
  
  Serial.begin(9600); // start serial communication with host
  
  //-- Servo hardware init
  innerPlate.attach(innerPlatePin);
  outerPlate.attach(outterPlatePin);

  //-- Set the servos to zero pint;
  innerPlate.writeMicroseconds(Offset[0]);
  outerPlate.writeMicroseconds(Offset[1]);

}

/*********************************************************************************
 **
 **  The main loop.  This loop runs continuously on the Arduino
 **
 *********************************************************************************/
void loop()
{ 
  Loop_Timing();
  Read_Serial_Port(); // check for new setpoint
  Update_Servos();  // write values to the servos
  
  if (loop_number >= update_rate) // only report values back to host every 10 loop
  {
     Serial.print("S: ");
     Serial.print(servo[0]); // send the inner angle
     Serial.print(" ");
     Serial.print(servo[1]); // send the outer angle
     Serial.print(" :E");
     
     loop_number = 1;
  }
  else
  {
    loop_number = loop_number + 1;
  }
}

/*********************************************************************************
 **
 **  Functions
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
