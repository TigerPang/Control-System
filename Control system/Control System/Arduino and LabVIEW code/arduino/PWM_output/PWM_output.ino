// Simple Pulse Width Modulation (analog) output example to control a motor
// Russell SA Brinkworth, 2014
// Yee Wei Law, 2015

int pwm_out = 8; // define where the pwm output is located. In this case it is connected to pin 8
float duty_cycle = 0.31; // min: 0.31, max: 1
int loop_period = 100; // loop delay to set the update rate

void setup()
{
  pinMode(pwm_out, OUTPUT);   // sets the pin as output
  analogWrite(pwm_out, 255); // large starting torque
}

void loop() // start main loop
{
  delay(loop_period); // pause the program so as to keep timing
  
  analogWrite(pwm_out, int(255*duty_cycle)); // send value to output
}
