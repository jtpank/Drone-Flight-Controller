#include<Wire.h>
#include <avr/interrupt.h>

//Global variables
int escPulse1 = 0;
int escPulse2 = 0;
int escPulse3 = 0;
int escPulse4 = 0;
int escThrottle = 0;

int startMotors;

//values may change at any point from Interrupt routines
//receiver inputs
// 1 --> left/right RJ (ROLL) | 2 --> up/down RJ (PITCH) | 3 --> up/down LJ (THROTTLE) | 4 --> left/right LJ (YAW)
//raw receiver INPUTS
volatile int receiverInput1, receiverInput2, receiverInput3, receiverInput4;
//interrupt timer current time
volatile unsigned long interruptTimerCT;
unsigned long pulseTimer1, pulseTimer2, pulseTimer3, pulseTimer4, escTimer;


//Channel timers for output
volatile unsigned long channelTimer1, channelTimer2, channelTimer3, channelTimer4;

unsigned long recIn[4];
//Values for prior pin state
volatile int latestChannel1 = 0;
volatile int latestChannel2 = 0;
volatile int latestChannel3 = 0; 
volatile int latestChannel4 = 0;


//Outputs to ESCs
int escOutputs[4];
unsigned long currentTime, loopTimer, eTimer, iTimer;
void pulseESC_OUTPUT();


void setup() {
  //Serial for debugging only
  //Serial.begin(115200);
  Wire.begin();   //I2C comm, master
  TWBR = 12;      //I2C clock, 400khz
  //TODO: configure digital pins 4,5,6,7,12,13 as outputs
  //set registers
  DDRD |= B11110000;
  DDRB |= B00110000;

  cli(); //stop interrupts
  //enable interrupts
  PCICR |= B00000001;
  //choose which pins to interrupt
  PCMSK0 |= B00001111;
  sei(); //allow interrupts
  loopTimer = micros();
  startMotors = 2;
}

void loop() {

  //Serial.print("Start Motors value: ");
  //Serial.println(startMotors);

  //STARTUP ROUTINE:
  // first: throttle low, yaw left
  //if((int) receiverInput3 < 1050 && (int) receiverInput4 < 1050) {startMotors = 1;}
  //when start 1, move yaw back to middle and start motors
  //if(startMotors == 1 && (int) receiverInput3 < 1050 && (int) receiverInput4 > 1450) {startMotors = 2;}
  //to stop the motors, throttle low, yaw right
  //if(startMotors == 2 && (int) receiverInput3 < 1050 && receiverInput4 > 1950) {startMotors = 0;} 

  //TODO: PID control sequence
  
  
  escThrottle = (int) receiverInput3; 

  if(startMotors == 2)
  {
    //full control of throttle:
    if(escThrottle > 1750) {escThrottle = 1750;}

    //for now, motors run on the same pulse
    escPulse1 = escThrottle;
    escPulse2 = escThrottle;
    escPulse3 = escThrottle;
    escPulse4 = escThrottle;


    
    if(escPulse1 < 1100) escPulse1 = 1100; //lower bound 1100 us
    if(escPulse1 > 1950) escPulse1 = 1950; //upper bound 1950 us
    if(escPulse2 < 1100) escPulse2 = 1100; 
    if(escPulse2 > 1950) escPulse2 = 1950; 
    if(escPulse3 < 1100) escPulse3 = 1100; 
    if(escPulse3 > 1950) escPulse3 = 1950; 
    if(escPulse4 < 1100) escPulse4 = 1100; 
    if(escPulse4 > 1950) escPulse4 = 1950;   
  }
  else
  {
    //if not started, pulse esc's so No Beeps
     escPulse1 = 1000;
     escPulse2 = 1000;
     escPulse3 = 1000;
     escPulse4 = 1000;
  }


  //250Hz refresh rate
  while(micros() - loopTimer < 4000);   
  loopTimer = micros();
  
  pulseESC_OUTPUT();

  



    
  
}

ISR(PCINT0_vect)
{
  interruptTimerCT = micros();
  //ch 1
  if(PINB & B00000001)
  {
    if(latestChannel1 == 0)
    {
       latestChannel1 = 1;
       channelTimer1 = interruptTimerCT;
    }
  }
  else if(latestChannel1 == 1)
  {
    latestChannel1 = 0;
    receiverInput1 = interruptTimerCT - channelTimer1;
  }
  
  //ch2
  if(PINB & B00000010)
  {
    if(latestChannel2 == 0)
    {
       latestChannel2 = 1;
       channelTimer2 = interruptTimerCT;
    }
  }
  else if(latestChannel2 == 1)
  {
    latestChannel2 = 0;
    receiverInput2 = interruptTimerCT - channelTimer2;
  }
  
  //ch 3
  if(PINB & B00000100)
  {
    if(latestChannel3 == 0)
    {
      latestChannel3 = 1;
      channelTimer3 = interruptTimerCT;
    }
  }
  else if(latestChannel3 == 1)
  {
     latestChannel3 = 0;
     receiverInput3 = interruptTimerCT - channelTimer3;
  }
  //ch 4
  if(PINB & B00001000)
  {
    if(latestChannel4 == 0)
    {
       latestChannel4 = 1;
       channelTimer4 = interruptTimerCT;
    }
  }
  else if(latestChannel4 == 1)
  {
    latestChannel4 = 0;
    receiverInput4 = interruptTimerCT - channelTimer4;
  }
  
}

void pulseESC_OUTPUT()
{
  loopTimer = micros();
  PORTD |= B11110000; //set outputs for escs to HIGH
  pulseTimer1 = escPulse1 + loopTimer;
  pulseTimer2 = escPulse2 + loopTimer;
  pulseTimer3 = escPulse3 + loopTimer;
  pulseTimer4 = escPulse4 + loopTimer;
  while(PORTD & B11110000)
  {
    escTimer = micros();
    if(pulseTimer1 <= escTimer){PORTD &= B11101111;}
    if(pulseTimer2 <= escTimer){PORTD &= B11011111;}
    if(pulseTimer3 <= escTimer){PORTD &= B10111111;}
    if(pulseTimer4 <= escTimer){PORTD &= B01111111;}
  }
}