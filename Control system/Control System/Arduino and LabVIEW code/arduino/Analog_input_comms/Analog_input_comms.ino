// Analog read and package data to put on serial port
// Russell SA Brinkworth, 2014

int analog_in = 0; // define where the analog input is located. In this case it is connected to analog pin 0
// ensure to connect the ground and +5V as reference
int val = 0; // variable to store the read value
int input_signal_length = 1; // number of data bytes to get from host per message
int input_scale = 4; // multiplication factor for input byte from host. Used to verify signal is being modified by Arduino when it is recieved
int incomingByte = 0; // for incoming serial data from host

boolean analog_input = false; // read from analog input (true) or from serial input (false)

int loop_period = 100; // loop delay to set the update rate

void setup()
{
  Serial.begin(9600); //  setup serial communications
}

void loop() // start main loop
{
  if (analog_input)
  {
    val = analogRead(analog_in); // read the input pin
  }
  else
  {
    while (Serial.available() > input_signal_length + 1) // check if there is a significant buffer backlog. Do not forget the start message byte
    {
      incomingByte = Serial.read();  // empty incomming buffer
    }
    if (Serial.available() >= input_signal_length + 1) // if serial data has the required number of bytes avaiable for the message
    {
      incomingByte = Serial.read();  // read from serial buffer
      if (incomingByte == 128) // see if the byte is the control byte for the start of a message
      {
        val = Serial.read() * input_scale;  // read first byte after start byte is recieved
      }
    }
  }
  
  Serial.print("S: "); // start charaters
  Serial.print(val); // print the value to serial port
  Serial.println(" :E"); // end characters and start new line
  delay(loop_period); // pause the program so as not to flood the system with new inputs
}
