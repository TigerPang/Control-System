// Simple Pulse Width Modulation (analog) output example to control a motor and record its speed based on a single channel encoder
// Russell SA Brinkworth, 2014
// Yee Wei Law, 2015

int pwm_out = 8; // define where the pwm output is located. In this case it is connected to pin 8
const float min_duty_cycle = 0.31;
const float max_duty_cycle = 1.00;
float duty_cycle = min_duty_cycle;
int analog_in = 0; // analog input pin
const float min_pot_val = 0;
const float max_pot_val = 1023;
int pot_val = 0; // value of potentiometer

volatile unsigned long ev_count = 0;
float motor_speed = 0.0;

int loop_period = 100; // loop delay to set the update rate

void setup()
{
  Serial.begin(9600); //  setup serial communications
  pinMode(pwm_out, OUTPUT); // sets the pin as output
  analogWrite(pwm_out, 255); // large starting torque
  attachInterrupt(0, event, CHANGE); // when interrupt 0 is triggered by a change in the value at pin 2 then run subrutine event
}

void loop() // start main loop
{
  delay(loop_period); // pause the program so as to keep timing
  
  // read potentiometer value
  pot_val = analogRead(analog_in);
  Serial.print("pot_val = "); Serial.print(pot_val); 
  
  // output PWM duty cycle
  duty_cycle = (max_duty_cycle - min_duty_cycle)/(max_pot_val - min_pot_val)*(pot_val - min_pot_val) + min_duty_cycle;
  analogWrite(pwm_out, int(255*duty_cycle));
  
  // estimate motor speed
  motor_speed = (ev_count/10)/(0.001*loop_period)*60; // 10 events per revolution
  Serial.print("\t\trpm = "); Serial.println(motor_speed);
  ev_count = 0;
}

void event() // triggered on interrupt
{
  ++ev_count;
}
