#include <Wire.h>                          //Gyro comm I2C


//Global variables
//const int 0x68 = 0x68;


//TODO enter in the variables after testing
//PID variables for roll, pitch, and yaw calculations
float pRoll;
float iRoll;
float dRoll;
float outputRoll;
const int maxRoll = 400; //max output

float pPitch;
float iPitch;
float dPitch;
float outputPitch;
const int maxPitch = 400; //max output

float pYaw;
float iYaw;
float dYaw;
float outputYaw;
const int maxYaw = 400; //max output

//PID setpoints
float pidRoll, iLastRoll, rollSetpoint, dErrorRoll;
float pidPitch, iLastPitch, pitchSetpoint, dErrorPitch;
float pidYaw, iLastYaw, yawSetpoint, dErrorYaw;
float gyroRollInput, gyroPitchInput, gyroYawInput;



//number of calibration inputs to calculate calibration average for zero-ing the gyro.
const int CALIBRATION_N = 2000;
//Complementary filter such that COMP_A + COMP_B = 1
const float COMP_A = 0.95;
const float COMP_B = 0.05;
int accAxis[3], gyroAxis[3], calibrationAvg, battVolatage;
float accRollAngle, accPitchAngle, pitchAngle, rollAngle; 
double gyroCalibration[3], gyroPitch, gyroRoll, gyroYaw;
long accX, accY, accZ, accVecMag;

int escPulse1 = 0;
int escPulse2 = 0;
int escPulse3 = 0;
int escPulse4 = 0;
int escThrottle = 0;

unsigned long mainLoopTimer;
int startMotors = 0;

//Other functions declarations
void readMPUOutput();
void setGyroRegisters();
void resetPID();
void calculatePID();

void setup() {
    Serial.begin(38400);
    Wire.begin(); //I2C comm, master
    TWBR = 12; //I2C clock, 400khz

    //Serial.println("Setting mpu registers...");
    setGyroRegisters();
    //Wait before continue

    //Calibrate the gyro
    //Serial.println("Calibrating gyro...");
    for(int c = 0; c<CALIBRATION_N; c++)
    {
      readMPUOutput();
      gyroCalibration[0] += gyroAxis[0];
      gyroCalibration[1] += gyroAxis[1];
      gyroCalibration[2] += gyroAxis[2];
    }
    //take average and store in gyroCalibration variable
    for(int c = 0; c<3; c++)
    {
      gyroCalibration[c]/= CALIBRATION_N;
    }
    
    //Serial.println("Starting main loop...");
}

void loop() {
  Serial.print(pitchAngle);
  Serial.print(",");
  Serial.println(rollAngle-4);
  //Serial.print(gyroPitch);
  //Serial.print(",");
  //Serial.println(gyroRoll);
  //Serial.print(accPitchAngle);
  //Serial.print(",");
  //Serial.println(accRollAngle);
  //Serial.print(accX);
  //Serial.print(",");
  //Serial.print(accY);
  //Serial.print(",");
  //Serial.println(accZ);

  if(startMotors == 1)
  {
    resetPID();
  }
  
  //0.0000611 = 1 / (250Hz / 65.5)
  //integrate angle per timestep
  pitchAngle+=gyroPitch * 0.0000611;   
  rollAngle+=gyroRoll  * 0.0000611;   
  //Yaw transfer
  //convert to degrees from radians
  // 0.000001066 = 1/(250/65.5) * PI/180
  
  pitchAngle -= rollAngle * sin(gyroYaw * 0.000001066);   
  rollAngle += pitchAngle * sin(gyroYaw * 0.000001066);  
  //acclerometer vector 
  accVecMag = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));
  
 
                                    
  accRollAngle = atan2(-accX,accZ)*60;    //Calculate the roll angle.                             
  accPitchAngle = atan2(accY,sqrt(accX*accX+accZ*accZ))*60;     //Calculate the pitch angle.
  
   

   //SET PID SETPOINTS
   //CALCULATE PID
   //ASSIGN ESC PULSE VALUES
  calculatePID();
  if(startMotors == 2)
  {
    escPulse1 = escThrottle - outputPitch + outputRoll - outputYaw; //Calculate the pulse for esc 1
    escPulse2 = escThrottle + outputPitch + outputRoll + outputYaw; //Calculate the pulse for esc 2 
    escPulse3 = escThrottle + outputPitch - outputRoll - outputYaw; //Calculate the pulse for esc 3 
    escPulse4 = escThrottle - outputPitch - outputRoll + outputYaw; //Calculate the pulse for esc 4
  }
 
                                                                                          
  

  //COMP_A and COMP_B are complementary filter values
  
  pitchAngle = pitchAngle*COMP_A + accPitchAngle*COMP_B;
  rollAngle = rollAngle*COMP_A + accRollAngle*COMP_B;
  
  
  readMPUOutput();
  while(micros() - mainLoopTimer < 4000){}//250Hz so wait until 4000us are passed.
  mainLoopTimer = micros();                                                   
}


void readMPUOutput()
{
    Wire.beginTransmission(0x68);  //Comm with adddress 0x68
    Wire.write(0x3B);  //Start reading at register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true); //request 6 bytes
    accAxis[0] = Wire.read() << 8 | Wire.read(); //read lo and hi byte to accX
    accAxis[1] = Wire.read() << 8 | Wire.read(); //read lo and hi byte to accY
    accAxis[2] = Wire.read() << 8 | Wire.read(); //read lo and hi byte to accZ

    accX = (long)accAxis[0]; 
    accY = (long)accAxis[1];
    accZ = (long)accAxis[2];
    
    Wire.beginTransmission(0x68);  //Comm with adddress 0x68
    Wire.write(0x43);  //Start reading at register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true); //request 6 bytes
    gyroAxis[0] = Wire.read() << 8 | Wire.read(); //read lo and hi byte to gyroX
    gyroAxis[1] = Wire.read() << 8 | Wire.read(); //read lo and hi byte to gyroY
    gyroAxis[2] = Wire.read() << 8 | Wire.read(); //read lo and hi byte to gyroZ

    gyroRoll = (float)gyroAxis[0]/65.5;
    gyroPitch = (float)gyroAxis[1]/ 65.5;
    gyroYaw = (float)gyroAxis[2]/ 65.5;
   
    if (calibrationAvg == CALIBRATION_N)
    {
        for (int i = 0; i < 4; i++)
        {
            gyroAxis[i] -= gyroCalibration[i];
        }
    }
}

void setGyroRegisters()
{
    Wire.beginTransmission(0x68);//Comm with adddress 0x68 ---------------------------------------------------------------------------
    Wire.write(0x6B);//Write to the PWR_MGMT_1 register (6B hex)
    Wire.write(B00000000); //Set bits: 00000000 for activating gyro
    Wire.endTransmission(true);
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); //Write to the GYRO_CONFIG register (1B hex)
    Wire.write(B00010000); //Set bits: 00001000 FSEL 500 (degrees per second)
    Wire.endTransmission(true);

    Wire.beginTransmission(0x68);
    Wire.write(0x1C); //Write to the ACCEL_CONFIG register (1C hex)
    Wire.write(B00010000); //Set bits: 00010000 for FSEL +/- 8g 
    //B00011000
    Wire.endTransmission(true);
}


void resetPID()
{
  iLastRoll = 0;
  dErrorRoll = 0;
  iLastPitch = 0;
  dErrorPitch = 0;
  iLastYaw = 0;
  dErrorYaw = 0;
  
}

void calculatePID()
{
  //roll
  float pidError = gyroRollInput - rollSetpoint;
  iLastRoll += iRoll*pidError;
  if(iLastRoll > maxRoll) {iLastRoll = maxRoll;}
  else if(iLastRoll < maxRoll*-1) {iLastRoll = maxRoll*-1;}
  //actual calc
  outputRoll = pRoll*pidError + iLastRoll + dRoll*(pidError-dErrorRoll);
  if(outputRoll > maxRoll) {outputRoll = maxRoll;}
  else if(outputRoll < maxRoll*-1) {outputRoll = maxRoll*-1;}
  //set new d val
  dErrorRoll = pidError;

  //pitch
  pidError = gyroPitchInput - pitchSetpoint;
  iLastPitch += iPitch*pidError;
  if(iLastPitch > maxPitch) {iLastPitch = maxPitch;}
  else if(iLastPitch < maxPitch*-1) {iLastPitch = maxPitch*-1;}
  //actual calc
  outputPitch = pPitch*pidError + iLastPitch + dPitch*(pidError-dErrorPitch);
  if(outputPitch > maxPitch) {outputPitch = maxPitch;}
  else if(outputPitch < maxPitch*-1) {outputPitch = maxPitch*-1;}
  //set new d val
  dErrorPitch = pidError;


  //yaw
  pidError = gyroYawInput - yawSetpoint;
  iLastYaw += iYaw*pidError;
  if(iLastYaw > maxYaw) {iLastYaw = maxYaw;}
  else if(iLastYaw < maxYaw*-1) {iLastYaw = maxYaw*-1;}
  //actual calc
  outputYaw = pYaw*pidError + iLastYaw + dYaw*(pidError-dErrorYaw);
  if(outputYaw > maxYaw) {outputYaw = maxYaw;}
  else if(outputYaw < maxYaw*-1) {outputYaw = maxYaw*-1;}
  //set new d val
  dErrorYaw = pidError;
  
  
    
}