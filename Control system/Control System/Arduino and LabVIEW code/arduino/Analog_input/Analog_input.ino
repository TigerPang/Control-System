// Simple analog read and print to serial port example
// Russell SA Brinkworth, 2014

int analog_in = 0; // define where the analog input is located. In this case it is connected to analog pin 0
// ensure to connect the ground and +5V as reference
int val = 0; // variable to store the read value
int loop_period = 100; // loop delay to set the update rate

void setup()
{
  Serial.begin(9600); //  setup serial communications
}

void loop() // start main loop
{
  val = analogRead(analog_in); // read the input pin
  Serial.println(val); // print the value to serial port and end with a new line
  delay(loop_period); // pause the program so as not to flood the system with new inputs
}
