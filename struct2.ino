#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

bool blinkState = false;


#define BUZZER_HERZ 131 
#define PITCH_HERZ 500
#define YAW_HERZ 100
#define HEIGHT 500

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

  float start_yaw=0.0, start_pitch = 0.0;
  float can = 20.0;
  
  float yaw=0;
  static int wrong_count_yaw=0;
  static int count_yaw = 0;
  static int left_count_yaw=0;
  static int right_count_yaw=0;
  float avg_left_yaw=0.0;
  float avg_right_yaw=0.0;
  int ring_yaw=0;

  float pitch=0;
  static int wrong_count_pitch=0;
  static int count_pitch = 0;
  static int left_count_pitch=0;
  static int right_count_pitch=0;
  float avg_left_pitch=0.0;
  float avg_right_pitch=0.0;
  int ring_pitch=0;

int pin_Alarm = 8;
int pin_Control = 9; 
int sound_Vibration = 0;
int control_Count = 0;
int measure_Count = 0;
unsigned long curtime = 0;
unsigned long pretime = 0;
int Rbuzzer = 5;
int Lbuzzer = 6;
int Rmotor = 10;
int Lmotor = 11;

LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {
      // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

// button control

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready

        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 
  pinMode(pin_Alarm, INPUT);
  pinMode(pin_Control, INPUT);
  pinMode(Rbuzzer, OUTPUT);
  pinMode(Lbuzzer, OUTPUT);
  pinMode(Rmotor, OUTPUT);
  pinMode(Lmotor, OUTPUT);
}

void loop() {
  
  if(digitalRead(pin_Alarm)==1)
  {
    sound_Vibration++;
    delay(400);
  }
  if(digitalRead(pin_Control)==1)
  {
    control_Count++;
    delay(400);
  }

  if(control_Count % 3 == 1)
  {
    if(measure_Count == 0)
    {
      for(int i=0; i<10; i++)
      {
        tone(Rbuzzer, BUZZER_HERZ);
        tone(Lbuzzer, BUZZER_HERZ);
        delay(400);
        noTone(Rbuzzer);
        noTone(Lbuzzer);
        delay(400);
      }
      if (!dmpReady) return;
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
    }
      start_yaw=ypr[0]* 180/M_PI;
      start_pitch=ypr[1]* 180/M_PI;
    }
    measure_Count=1;
    
 
 if (!dmpReady) return;
 if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
    }
//yaw(수평기울기) 측정
    
yaw = ypr[0]* 180/M_PI;
  if(yaw<start_yaw){
    left_count_yaw++;

    avg_left_yaw+=(yaw-avg_left_yaw)/float(left_count_yaw);

    if (yaw < start_yaw - can) { // 왼쪽
              wrong_count_yaw ++;

              if (wrong_count_yaw > 5) {
                ring_yaw = 1;
                wrong_count_yaw = 0;

              }
            }
             else{
              ring_yaw=0;
            }
  }
  
  else{
    right_count_yaw++;

    avg_right_yaw+=(yaw-avg_right_yaw)/float(right_count_yaw);

    if (yaw > start_yaw + can) { // 오른쪽
              wrong_count_yaw ++;

              if (wrong_count_yaw > 5) {
                ring_yaw = 1;
                wrong_count_yaw = 0;
              }
            }
             else{
              ring_yaw=0;
            }
            
  }
  

//pitch(수직기울기) 측정
  pitch = ypr[1]* 180/M_PI;
  if(pitch<start_pitch){
    left_count_pitch++;

    avg_left_pitch+=(pitch-avg_left_pitch)/float(left_count_pitch);

    if (pitch < start_pitch - can) { // 왼쪽
              wrong_count_pitch ++;

              if (wrong_count_pitch > 5) {
                ring_pitch = 1;
                wrong_count_pitch = 0;

              }
            }
            else{
              ring_pitch=0;
            }
            
  }
  
  else{
    right_count_pitch++;

    avg_right_pitch+=(pitch-avg_right_pitch)/float(right_count_pitch);

    if (pitch > start_pitch + can) { // 오른쪽
              wrong_count_pitch ++;

              if (wrong_count_pitch > 5) {
                ring_pitch = 1;
                wrong_count_pitch = 0;
              }
            }
             else{
              ring_pitch=0;
            }
            
  }
  Serial.println(start_pitch);
  Serial.println(pitch);
  Serial.println(ring_pitch);
  



    if(ring_pitch == 1)
    {
      
      switch(sound_Vibration % 4){
        case 1:
          if(pitch >= start_pitch)
          {
            
            tone(Rbuzzer, PITCH_HERZ);
           
          }
          else if(pitch < start_pitch)
          {
            tone(Lbuzzer, PITCH_HERZ);
       
          }
          break;
          
        case 2:
          if(pitch >= start_pitch)
          {
            digitalWrite(Rmotor, HIGH);
            tone(Rbuzzer, PITCH_HERZ);
          
          }
          else if(pitch < start_pitch)
          {
            digitalWrite(Lmotor, HIGH);
            tone(Lbuzzer, PITCH_HERZ);
           
          }
          break;
       
        case 3:
          if(pitch >= start_pitch)
          {
            digitalWrite(Rmotor, HIGH);
           
          }
          else if(pitch < start_pitch)
          {
            digitalWrite(Lmotor, HIGH);
          
          }
          break; 
      }
    }
                 
      
    else if(ring_yaw == 1)
    {
      switch(sound_Vibration % 4){
        case 1:
          if(yaw >= start_yaw)
          {
            tone(Rbuzzer, YAW_HERZ);
          }
          else if(yaw < start_yaw)
          {
            tone(Lbuzzer, YAW_HERZ);

          }
          break;
          
        case 2:
          if(yaw >= start_yaw)
          {
            tone(Rbuzzer, YAW_HERZ);
            digitalWrite(Rmotor, HIGH);
          
          }
          else if(yaw < start_yaw)
          {
            tone(Lbuzzer, YAW_HERZ);
            digitalWrite(Lmotor, HIGH);
         
          }
          break;
       
        case 3:
          if(yaw >= start_yaw)
          {
            digitalWrite(Rmotor, HIGH);
        
          }
          else if(yaw < start_yaw)
          {
            digitalWrite(Lmotor, HIGH);
            
          }
          break;
        default : 
          break;          
      }
      
    }
     
    else
    {
      noTone(Rbuzzer);
      noTone(Lbuzzer);
      digitalWrite(Rmotor, LOW);
      digitalWrite(Lmotor, LOW);
    }
  }
  else if(control_Count % 3 == 2)
  {
    noTone(Rbuzzer);
    noTone(Lbuzzer);
    digitalWrite(Rmotor, LOW);
    digitalWrite(Lmotor, LOW);
    measure_Count=0; 
  }
}
