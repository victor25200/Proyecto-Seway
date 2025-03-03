# Proyecto-Seway
## Código 
### self_valancing_robot_V1.ino
```cpp
/*MOTOR DRIVER VARIABLES AND CONFIGURATION--------------------------------------------------------------------------------*/
// 27 - Standby pin
// 14 - AIN1 pin
// 12 - AIN2 pin
// 13 - PWMA pin
// 26 - BIN1 pin
// 25 - BIN2 pin
// 33 - PWMB pin
// To reverse forward motor direction, switch the AIN1 and AIN2 or BIN1 and BIN2 pin numbers.
const int pwm_left_motor = 13; //enableA    
const int pwm_right_motor = 33;//enable B
const int standby = 27;
const int motor_left_AIN1 = 14;
const int motor_left_AIN2 = 12;
const int motor_right_BIN1 = 25;
const int motor_right_BIN2 = 26;

const int pwm_frecuency = 980;           // PWM frecuency for arduino uno 
const int pwm_left_channel = 0;          // 0-15 available channels (left motor channel) 
const int pwm_right_channel = 1;         // 0-15 available channels (right motor channel)
const int resolution = 8;                // 8-bit resolution means control values from 0 to 255

/*--------------------------------------------------------------------------------------------------------------------------*/

/*ANALOG INPUT VARIABLES AND CONFIGURATION(POTENTIOMETERS)------------------------------------------------------------------*/
//ADC2 (GPIOS 2,4,12-15,25-27) cannot be used when Wi-Fi is used. GPIOS 34-35 belong to ADC1 (GPIOS 32-39)
const int potA = 34;                     //Potentiometer 5k ohm (orange head)
const int potB = 35;                     //Potentiometer 5k ohm (blue head)
float potA_value =0;                     //variable for storing the potentiometer A value(0-4095)
float potB_value =0;                     //variable for storing the potentiometer B value(0-4095)

      
/*--------------------------------------------------------------------------------------------------------------------------*/


/*MPU6050 VARIABLES AND CONFIGURATION--------------------------------------------------------------------------------------*/ 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  /* Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
  is used in I2Cdev.h*/
  #include "Wire.h"
#endif

MPU6050 mpu;                             //class default I2C address is 0x68. Use MPU6050 mpu(0x69) for ADO high 
#define OUTPUT_READABLE_YAWPITCHROLL     // pitch/roll angles (in degrees) calculated from the quaternions coming from FIFO
#define INTERRUPT_PIN 23                 // use the interrupt pin in MPU6050

// MPU control/status variables
bool dmpReady = false;                   // set true if DMP init was successful
uint8_t mpuIntStatus;                    // holds actual interrupt status byte from MPU
uint8_t devStatus;                       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                  // FIFO storage buffer

//orientation/motion variabless
Quaternion q;
VectorFloat gravity;
float ypr[3];                            // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
                                         // packet structure for InvenSense teapot demo
                                        
volatile bool mpuInterrupt = false;      // Interrupt detection routine. Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float pitch = 0;                         //variable to catch pitch data in degress from IMU 

/*--------------------------------------------------------------------------------------------------------------------------*/


/*INTERNAL ESP32 LED VARIABLES AND CONFIGURATION----------------------------------------------------------------------------*/
int cont = 0;                            //cont for defined blinks
unsigned long currentTime=0;             //current time from esp32 in millis
unsigned long previousTime=0;            //previous time from esp32 in millis
bool ledState=LOW;                       //initial state of internal LED

/*--------------------------------------------------------------------------------------------------------------------------*/


/*PID VARIABLES AND CONFIGURATION-------------------------------------------------------------------------------------------*/
#include "PID_v1.h"
double kp = 18.67;
double ki = 0243.67;
double kd = 0;
double setpoint, input, output;         
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

/*--------------------------------------------------------------------------------------------------------------------------*/


/*VOID SETUP CONFIGURATION--------------------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Setup_motors();
  Setup_MPU6050();
  Setup_PID();
}
/*--------------------------------------------------------------------------------------------------------------------------*/


/*LOOP PROGRAM--------------------------------------------------------------------------------------------------------------*/
void loop() {
  
  potB_value = float(map(analogRead(potB),0,4095,-300,300))/100;   //Blue potentiometer
  //potA_value = float(map(analogRead(potA),0,4095,0,3000))/10;  //Orange potentiometer
  /*erial.print("kp:");
  Serial.print(potB_value);
  Serial.print("\t");
  Serial.print("ki:");
  Serial.println(potA_value);
  */
  setpoint = 2.0;
  
  Update_MPU6050_DMP();       //read the IMU sensor
  pitch = ypr[1] * 180/M_PI;  //pitch value in degress
  input = pitch;              //PID control input variable
  
  myPID.SetSampleTime(5);           //sampling every 5ms
  /*kp = potB_value;                  //set pot.(blue) value float between 0-50 to kp
  ki = potA_value;                  //set pot.(orange)value float between 0-300 to ki        
  myPID.SetTunings(kp,ki,kd);       //set kp,ki,kd parameters
  */
  myPID.Compute();                  //PID control compute

 
  if (pitch >30 || pitch <-30 || pitch == setpoint){
    speed_motors(0);         //turn off the motors
  }
  else{
    speed_motors(output); //set the output PID control to motors
  }
}

/*--------------------------------------------------------------------------------------------------------------------------*/

```
## Setup_Configuration.ino
    
    /*SETUP MOTORS CONFIGURATION------------------------------------------------------------------------------------------------*/
    void Setup_motors(){
      pinMode(pwm_left_motor,OUTPUT);                                // declare GPIOS as output for write values
      pinMode(pwm_right_motor,OUTPUT);
      pinMode(standby,OUTPUT);
      pinMode(motor_left_AIN1,OUTPUT);
      pinMode(motor_left_AIN2,OUTPUT);
      pinMode(motor_right_BIN1,OUTPUT);
      pinMode(motor_right_BIN2,OUTPUT);
      
      //digitalWrite(standby,HIGH);                                     //turn on motor driver
      ledcSetup(pwm_left_channel, pwm_frecuency, resolution);         //configure pwm funtionalities for left motor  
      ledcSetup(pwm_right_channel, pwm_frecuency, resolution);        //configure pwm funtionalities for right motor
      ledcAttachPin(pwm_left_motor, pwm_left_channel);                //attach the channel to the gpio used for pwm left motor 
      ledcAttachPin(pwm_right_motor, pwm_right_channel);              //attach the channel to the gpio used for pwm right motor 
    }
    /*---------------------------------------------------------------------------------------------------------------------------*/
    
    
    
    /*SETUP MPU6050 CONFIGURATION------------------------------------------------------------------------------------------------*/
    void Setup_MPU6050()
    {
      pinMode(LED_BUILTIN, OUTPUT);                            //esp32 internal led
      pinMode(INTERRUPT_PIN, INPUT_PULLUP);                    //esp32 GPIO as external interrupt
      
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE         // join I2C bus (I2Cdev library doesn't do this automatically)
        Wire.begin();
        
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif
    
      
      mpu.initialize();
      //delay(100);
      if(mpu.testConnection() == true){
          blink_led(6,200);                                    //blink led 3 times(3 ON + 3 OFF = 6) every 200ms
      }
      devStatus = mpu.dmpInitialize();
     
      mpu.setXGyroOffset(5);                                 // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setYGyroOffset(-27);
      mpu.setZGyroOffset(7);
      mpu.setXAccelOffset(455);
      mpu.setYAccelOffset(271);
      mpu.setZAccelOffset(1210); 
    
      if(devStatus == 0){
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6); 
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
      }else{;} 
       
    }
    /*---------------------------------------------------------------------------------------------------------------------------*/
    
    /*SETUP PID------------------------------------------------------------------------------------------------------------------*/
    void Setup_PID(){
      myPID.SetOutputLimits(-175, 175);
      myPID.SetMode(AUTOMATIC);
      
### User_Funtions.ino
    /*MOTORS FUNCTIONS---------------------------------------------------------------------------------------------------------*/
    void speed_right_motor(int right)                         
    {
      if(right >= 0)                                    
      {
        digitalWrite(motor_right_BIN1,LOW);                  
        digitalWrite(motor_right_BIN2,HIGH);
        ledcWrite(pwm_right_channel, right);
      }
      
      if(right < 0)                                     
      {                            
        digitalWrite(motor_right_BIN1,HIGH);                  
        digitalWrite(motor_right_BIN2,LOW);
        ledcWrite(pwm_right_channel, abs(right));
      }
    }
    
    
    void speed_left_motor(int left)                         
    {
      if(left >= 0)                                    
      {
        digitalWrite(motor_left_AIN1,LOW);                  
        digitalWrite(motor_left_AIN2,HIGH);
        ledcWrite(pwm_left_channel, left);
      }
      
      if(left < 0)                                     
      {                            
        digitalWrite(motor_left_AIN1,HIGH);                  
        digitalWrite(motor_left_AIN2,LOW);
        ledcWrite(pwm_left_channel, abs(left));
      }
    }
    
    
    void speed_motors(int robot_speed)                         
    {
      if(robot_speed >= 0)                                    
      {
        digitalWrite(motor_left_AIN1,LOW);                  
        digitalWrite(motor_left_AIN2,HIGH);
        digitalWrite(motor_right_BIN1,LOW);                  
        digitalWrite(motor_right_BIN2,HIGH);
        ledcWrite(pwm_right_channel, robot_speed);
        ledcWrite(pwm_left_channel, robot_speed);
      }
      
      if(robot_speed < 0)                                     
      {                            
        digitalWrite(motor_left_AIN1,HIGH);                  
        digitalWrite(motor_left_AIN2,LOW);
        digitalWrite(motor_right_BIN1,HIGH);                  
        digitalWrite(motor_right_BIN2,LOW);
        ledcWrite(pwm_right_channel, abs(robot_speed));
        ledcWrite(pwm_left_channel, abs(robot_speed));
      }
    }
    /*---------------------------------------------------------------------------------------------------------------------------*/
    
    
    /*MPU6050 FUNCTIONS----------------------------------------------------------------------------------------------------------*/
    
    void Update_MPU6050_DMP(){
      if (!dmpReady) return;                                  // if programming failed, don't try to do anything
      while (!mpuInterrupt && fifoCount < packetSize) {       // wait for MPU interrupt or extra packet(s) available
        if (mpuInterrupt && fifoCount < packetSize) { 
          fifoCount = mpu.getFIFOCount();                     // try to get out of the infinite loop
        }  
      }
      
      mpuInterrupt = false;                                   // reset interrupt flag and get INT_STATUS byte
      mpuIntStatus = mpu.getIntStatus();
      fifoCount = mpu.getFIFOCount();                         //get current FIFO count
        
      if ((mpuIntStatus & 0x10) || fifoCount > 512) {
        mpu.resetFIFO();                                      // reset so we can continue cleanly
      }
      else if (mpuIntStatus & 0x02) {                         // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
          mpu.getFIFOBytes(fifoBuffer, packetSize);           // read a packet from FIFO
          fifoCount -= packetSize;                            // (this lets us immediately read more without waiting for an interrupt)
          
          #ifdef OUTPUT_READABLE_YAWPITCHROLL                // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            
          #endif
       }    
    }
    /*---------------------------------------------------------------------------------------------------------------------------*/
    
    
    /*BLINK INTERNAL LED FUNCTION------------------------------------------------------------------------------------------------*/
    void blink_led(int n_blinks, int time_ms){
      while (cont <= n_blinks){
        currentTime=millis();
        if((currentTime-previousTime)>time_ms){
          previousTime=currentTime;
          ledState=!ledState;
          digitalWrite(LED_BUILTIN,!ledState);
          cont+=1;
        }
      }
    }
    /*---------------------------------------------------------------------------------------------------------------------------*/
    
## Diseños
