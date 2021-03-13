#include <Wire.h>                          //Gyro comm I2C
#include <EEPROM.h>                        //Store on EEPROM


//Global variables
int escPulse1, escPulse2, escPulse3, escPulse4;
//values may change at any point from Interrupt routines
//receiver inputs
// 1 --> left/right RJ | 2 --> up/down RJ | 3 --> Throttle up/down LJ | 4 --> left/right LJ
//raw receiver INPUTS
volatile int receiverInput1, receiverInput2, receiverInput3, receiverInput4;

//Values for prior pin state
int latestChannel, latestChannel2, latestChannel3, latestChannel4;
//Channel timers for output
unsigned long channelTimer1, channelTimer2, channelTimer3, channelTimer4;
//Outputs to ESCs
int escOutputs[4], escThrottle;

//TODO: change to 0 for start routine
int startMotors = 2;


unsigned long currentTime, loopTimer, eTimer;

//Other function declarations
void pulseESC(void);


void setup() {
  Wire.begin();   //I2C comm, master
  TWBR = 12;      //I2C clock, 400khz
  //TODO: configure digital pins 4,5,6,7,12,13 as outputs
  //set registers
  DDRD |= B11110000;
  DDRB |= B00110000;

  //set interrupt pins
  //digital pins 8,9,10,11 to trigger interrupt on change
  //as per atmega328 datasheet
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  
  //TODO: gyro calibrations
  //NOTE: in gyro calib, call
  pulseESC_NOBEEP();
  //Pulse esc's so no beeping
 
  
  //TODO: Wait for transmitter to set at lowest position
  //convert receiver signals to 1000-2000us pulse
  //NOTE: in transmitter calib, call
  // pulseAllESC();
  //Pulse esc's so no beeping

  //TODO: Determine battery voltage, and signal low battery
  // Use Analog Pin
  
  

}

void loop() {

  //Motor start with low throttle, yaw left (prelude)
  //if(receiverInput3 < 1050 && receiverInput4<1050){ startMotors = 1;}
  //yaw back to center starts motors
  //if(startMotors == 1 && receiverInput3 < 1050 && receiverInput4 > 1450){startMotors = 2;}
  //TODO: reset PID controls here
  
  //stopping motors
  //if(startMotors == 2 && receiverInput3 < 1050 && receiverInput4 > 1950){startMotors = 0;}
  //Motor control with controller (NO AUTO LEVEL)
  if(startMotors == 2)
  {
    escPulse1 = escThrottle; //receiverInput3 is Throttle;

    if(escPulse1 < 1100 && startMotors == 2)
    {
       escPulse1 = 1100;
    }
    else if(escPulse1 > 2000 && startMotors == 2)
    {
      escPulse1 = 2000;
    }
    else if(startMotors != 2)
    {
      //prevent beeping
      escPulse1 = 1000;
    }
  }

  while(micros() - loopTimer < 4000);   
  loopTimer = micros();

  pulseESC_OUTPUT();
  
}


//Other functions
void pulseESC_NOBEEP()
{
  PORTD |= B11110000;//Set digital port 4, 5, 6, 7 high.
  delayMicroseconds(1000);//Wait 1000us.
  PORTD &= B00001111;//Set digital port 4, 5, 6, 7 low.
  delay(2);
}

//pulsing esc pins
void pulseESC_OUTPUT()
{
  loopTimer = micros();
  PORTD |= B11110000; //set outputs for escs to HIGH
  channelTimer1 = escPulse1 + loopTimer;

  while(PORTD>=16)
  {
    eTimer = micros();
    //digital port 4
    if(channelTimer1 <= eTimer){  PORTD &= B11101111; }
  }
}

//TODO: Setup intterupt service routines
ISR(PCINT0_vect)
{
  currentTime = micros();
  //CHANNEL 3 (THROTTLE) up/down LJ
  //Is input 10 high?
  if(PINB & B00000100)
  {
    //if last time it was LOW signal, then we have a new pulse, start timer
    if(latestChannel3 == 0)
    {
      //set latestChannel3 to HIGH
      latestChannel3 == 1;
      //set channelTimer3 to currentTime
      channelTimer3 = currentTime;
    }
  }
  //otherwise input 10 is LOW and changed from HIGH to LOW, stop timer count, and send throttle pulse to ESC
  else if(latestChannel3 == 1)
  {
    //set latestChannel3 to LOW
    latestChannel3 == 0;
    //set new output Pulse duration to ESC
    escThrottle = currentTime - channelTimer3;
    
  }
    
}
