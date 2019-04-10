#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

#define PIN_INPUT A0
#define PIN_FEEDBACK A1
#define PIN_OUTPUT 3

const int numReadings = 10;
const int nilaiPWMmin = 228;
const int nilaiPWMmax = 426;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;
int total = 0;                  // the running total

modbusDevice regBank;
modbusSlave slave;

double Feedback, pwmOut, tunner=0;

bool mode=1;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp, Ki, Kd;

float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;
int PID_values_fixed =0;

void setup()
{
  //Adding Modbus Memory
  regBank.add(1);
  regBank.add(40001);
  regBank.add(40002);
  regBank.add(40003);
  regBank.add(40004);
  regBank.add(40005);
  regBank.add(40006);
  regBank.add(40007);
  regBank.add(40008);
  regBank.add(40009);
  regBank.add(40010);

  //Set modbus slave
  slave._device = &regBank;  
  slave.setBaud(9600); 
  regBank.setId(1);
   
  //initialize the variables we're linked to
  Input = 0;
  Feedback = 0;
  Setpoint = 300;
  Kp=20, Ki=50, Kd=0;

  //initial value memory modbus
  regBank.set(1, (bool) mode);
  regBank.set(40001, (double) Kp);
  regBank.set(40002, (double) Ki);
  regBank.set(40003, (double) Kd);
  regBank.set(40004, (double) Setpoint);
  regBank.set(40005, (double) Input);
  regBank.set(40006, (double) Output);
  regBank.set(40007, (int) Feedback);

  //initialize PWM output
    // initialize all the readings to 0:
  DDRB |= (1 << DDB1) | (1 << DDB2);

  TCCR1A =
      (1 << COM1A1) | (1 << COM1B1) |
      // Fast PWM mode.
      (1 << WGM11);
TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
      // Fast PWM mode.
      
      // No clock prescaling (fastest possible
      // freq).
      (1 << CS10);
  OCR1A = 0;
  // Set the counter value that corresponds to
  // full duty cycle. For 15-bit PWM use
  // 0x7fff, etc. A lower value for ICR1 will
  // allow a faster PWM frequency.
  ICR1 = 0x03ff;
  /////////////////////////////////////////////////////////
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop()
{
  Get_Param();

  if(mode==1)
  {
    Auto();
  }
  else if (mode == 0) 
  {
    Man();
  }
  slave.run();
}

void Auto()
{
  // First we read the real value of temperature
  int Input_read = input_smoothing();
  //Next we calculate the error between the setpoint and the real value
  PID_error = Setpoint - Input_read;
  //Calculate the P value
  PID_p = 0.01*Kp * PID_error;
  //Calculate the I value in a range on +-3
  PID_i = 0.01*PID_i + (Ki * PID_error);
  

  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = 0.01*Kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  //Since we activate the MOSFET with a 0 to the base of the BJT, we write 255-PID value (inverted)
 
  Output = PID_value;
  previous_error = PID_error;
  Feedback = analogRead(PIN_FEEDBACK);
  input_smoothing();
  
  double fb = (Feedback+1)*100/1023;
  double cmd = (Output+1)*100/225;
  pwmOut=map(Output,0,255,nilaiPWMmin,nilaiPWMmax);
  if (cmd > fb){
    tunner++;
  }
  if (cmd < fb){
    tunner--;
  }
  if (cmd = fb){
    tunner=tunner;
  } 
  OCR1A=pwmOut+tunner/20; 
  pwmOut=map(Output,0,255,nilaiPWMmin,nilaiPWMmax);
  OCR1A=pwmOut;


  regBank.set(40006, (double) Output);
  regBank.set(40005, (double) Input);
  regBank.set(40007, (double) Feedback);
   
}

void Man()
{
  Feedback = analogRead(PIN_FEEDBACK);
  double fb = (Feedback+1)*100/1023;
  input_smoothing();
  Output = regBank.get(40006);
  double cmd = (Output+1)*100/225;
  pwmOut=map(Output,0,255,nilaiPWMmin,nilaiPWMmax);
  if (cmd > fb){
    tunner++;
  }
  if (cmd < fb){
    tunner--;
  }
  if (cmd = fb){
    tunner=tunner;
  } 
  OCR1A=pwmOut+tunner/20;

  regBank.set(40007, (double) Feedback);
  regBank.set(40005, (double) Input);
}

void Get_Param()
{
  mode=regBank.get(1);
  Kp=regBank.get(40001);
  Ki=regBank.get(40002);
  Kd=regBank.get(40003);
  Setpoint=regBank.get(40004);
}

double input_smoothing(){
    total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(PIN_INPUT);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  // calculate the average:
  Input = total / numReadings;
}
