#define version_0.65

#define DEBUGING
#define BLUE

#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <PID_v1.h>
#include <digitalIOPerformance.h>
#define DIGITALIO_NO_INTERRUPT_SAFETY
#define DIGITALIO_NO_MIX_ANALOGWRITE

//Bluetooth Stuff
#include<SoftwareSerial.h>

const int rxpin = 11; // pin used to receive (not used in this version) 
const int txpin = 4; // pin used to send to LCD

SoftwareSerial blue(rxpin, txpin); // new serial port on pins 2 and 3

#define RESTRICT_PITCH

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
// Balance PID controller Definitions
#define BALANCE_KP 15                   // PID Constants
#define BALANCE_KI 90
#define BALANCE_KD 0.8
#define BALANCE_PID_MIN -255              // Define PID limits to match PWM max in reverse and foward
#define BALANCE_PID_MAX 255

#define ROTATION_KP 50
#define ROTATION_KI 300
#define ROTATION_KD 4

#define MOTOR_A_DIR       9         //M11
#define MOTOR_A_BRAKE     8         //M12
#define MOTOR_B_DIR       10         //M21
#define MOTOR_B_BRAKE     12         //M22
#define MOTOR_A_PWM       5         //M1E
#define MOTOR_B_PWM       6         //M2E

// Motor Misc
#define PWM_MIN 0
#define PWM_MAX 255
float MOTORSLACK_A=32;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B=39;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
//#define MOTORSLACK_INC 1                  // Motor slack inc/dec value for slack compensation tuning
#define MOTOR_A_PWM_MAX 255               // Compensate for differences in DC motor strength
#define MOTOR_B_PWM_MAX 255               

int MotorAspeed, MotorBspeed, MotorSlack,moveState=0,dir_speed;
float prevA,prevB;

float yaw,input,out,setpoint,movingAngleOffset=1,originalSetpoint,Buffer[3],prevKp,prevKi,prevKd;
float yinput,yout,ysetpoint,yoriginalSetpoint;
bool set = true;
uint32_t timer,timer1;

PID pid(&input,&out,&setpoint,BALANCE_KP,BALANCE_KI,BALANCE_KD,DIRECT);
PID rot(&yinput,&yout,&ysetpoint,ROTATION_KP,ROTATION_KI,ROTATION_KD,DIRECT);

void setup()
{
  #ifdef DEBUGING
  Serial.begin(9600);
  #endif
  
  #ifdef BLUE
  blue.begin(9600);
  #endif
  
  init_imu();
  initmot();
  
pid.SetMode(AUTOMATIC);
pid.SetOutputLimits(-210, 210);
pid.SetSampleTime(10);
rot.SetMode(AUTOMATIC);
rot.SetOutputLimits(-20, 20);
rot.SetSampleTime(10);

setpoint = 0;
originalSetpoint = setpoint;
ysetpoint = 0;
yoriginalSetpoint = ysetpoint;
}


void loop() 
{
 // set_point();
  getvalues();
  new_pid();
 // stable();

#ifdef BLUE
Bt_control();
 //listen_bluetooth();
#endif

#ifdef DEBUGING
  printval();
#endif

}
    

void init_imu()
{
      // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
     mpu.initialize();
     devStatus = mpu.dmpInitialize();
     // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-9);
    mpu.setYGyroOffset(-3);
    mpu.setZGyroOffset(61);
    mpu.setXAccelOffset(-449);
    mpu.setYAccelOffset(2580);
    mpu.setZAccelOffset(1259);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
       // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}
    
void getvalues()
{
     // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
         } 
         yinput = ypr[0]* 180/M_PI;
         input = -ypr[1] * 180/M_PI;
}
    
void printval()
{
  Serial.print(yinput);Serial.print("\t"); 
  Serial.print(yoriginalSetpoint); Serial.print("\t");
  Serial.print(ysetpoint); Serial.print("\t");
  Serial.print(yout); Serial.print("\t");Serial.print("\t");  
  Serial.print(input);Serial.print("\t");
  Serial.print(originalSetpoint); Serial.print("\t");
  Serial.print(setpoint); Serial.print("\t");
  Serial.print(out); Serial.print("\t");Serial.print("\t");
  Serial.print(MotorAspeed); Serial.print("\t");
  Serial.print(MotorBspeed); Serial.println("\t");

}

void set_point()
{
   if(set)
  {
  while(millis()<25000)
  {
  getvalues();
  #ifdef DEBUGING
  printval();
  #endif
  }
  setpoint=input;
  originalSetpoint=setpoint;
  set = false;
  } 
}

void stable()
{
  if((millis()-timer)>=1000)
  {
    moveBackForth();
    timer=millis();
  }
}

void read_bluetooth()
{
 if(blue.available())
 {
  correct_pid();
  //correct_motors();
 }
}

void listen_bluetooth()
{
   if((millis()-timer1)>=100)
  {
    read_bluetooth();
  
    timer1=millis();
  }
}

void correct_pid()
{
 for(int i = 0;i<3;i++)
{
  Buffer[i]=blue.parseFloat();
} 
if(Buffer[0]!= prevKp || Buffer[1]!=prevKi || Buffer[2]!=prevKd)
{
  rot.SetTunings(Buffer[0],Buffer[1],Buffer[2]);
 
  prevKp=Buffer[0];
  prevKi=Buffer[1];
  prevKd=Buffer[2];
} 
}

void correct_motors()
{
 for(int i = 0;i<2;i++)
{
  Buffer[i]=blue.parseFloat();
} 
if(Buffer[0]!= prevA || Buffer[1]!=prevB)
{
 MOTORSLACK_A=Buffer[0];
 MOTORSLACK_B=Buffer[1];
  prevA=Buffer[0];
  prevB=Buffer[1];
} 
}

void Bt_control()
{
  if(blue.available())
  {
   char incomingByte=blue.read();
  switch(incomingByte)
 {
case 'F':setpoint = originalSetpoint - dir_speed;
break;
case 'B':setpoint = originalSetpoint + dir_speed;
break;
case 'L':ysetpoint = constrain((ysetpoint + yoriginalSetpoint - dir_speed),-180,180);
break;
case 'R':ysetpoint = constrain(ysetpoint + yoriginalSetpoint + dir_speed,-180,180);
break;
case '1':dir_speed=1;
break;
case '2':dir_speed=2;
break;
case '3':dir_speed=3;
break;
case '4':dir_speed=4;
break;
case '5':dir_speed=5;
break;
case '6':dir_speed=6;
break;
case '7':dir_speed=7;
break;
case '8':dir_speed=8;
break;
case '9':dir_speed=9;
break;
case 'q':dir_speed=10;
break;
default:setpoint=originalSetpoint;
 }
  }
}

double compensate_slack(double yOutput,double Output,bool A)
  {
   // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
  if(A)
  {
   if (Output >= 0) 
   Output = Output + MOTORSLACK_A - yOutput;
   if (Output < 0) 
   Output = Output - MOTORSLACK_A - yOutput;
  }
  else
  {
    if (Output >= 0) 
   Output = Output + MOTORSLACK_B + yOutput;
   if (Output < 0) 
   Output = Output - MOTORSLACK_B + yOutput;
  }
   Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX); 
  return Output;
}

void new_pid()
{
  
  pid.Compute();
  rot.Compute();
   // Convert PID output to motor control
   
     MotorAspeed = compensate_slack(yout,out,1);
     MotorBspeed = compensate_slack(yout,out,0);
     motorspeed(MotorAspeed, MotorBspeed);
}

void moveBackForth()
{
    moveState++;
    if (moveState > 2) moveState = 0;
    
    if (moveState == 0)
      setpoint = originalSetpoint;
    else if (moveState == 1)
      setpoint = originalSetpoint - movingAngleOffset;
    else
      setpoint = originalSetpoint + movingAngleOffset;
}

void initmot()
{
  //Pin definitions
    pinMode(MOTOR_A_DIR, OUTPUT);
    pinMode(MOTOR_A_BRAKE, OUTPUT);
    pinMode(MOTOR_B_DIR, OUTPUT);
    pinMode(MOTOR_B_BRAKE, OUTPUT);
    analogWrite(MOTOR_A_PWM, 0);
    analogWrite(MOTOR_B_PWM, 0);
}

// Motor control functions
void motorspeed(int MotorAspeed, int MotorBspeed) {
  // Motor A control
  if (MotorAspeed >= 0) 
  {
    digitalWrite(MOTOR_A_DIR,HIGH);
    digitalWrite(MOTOR_A_BRAKE,LOW);
  }
  else 
{
  digitalWrite(MOTOR_A_DIR,LOW);
  digitalWrite(MOTOR_A_BRAKE,HIGH);
}
  
  analogWrite(MOTOR_A_PWM,abs(MotorAspeed));

  // Motor B control
  if (MotorBspeed >= 0) 
  {
    digitalWrite(MOTOR_B_DIR,HIGH);
    digitalWrite(MOTOR_B_BRAKE,LOW);
  }
  else 
  {
  digitalWrite(MOTOR_B_DIR,LOW);
  digitalWrite(MOTOR_B_BRAKE,HIGH);
  }
  analogWrite(MOTOR_B_PWM, abs(MotorBspeed));
 
}
