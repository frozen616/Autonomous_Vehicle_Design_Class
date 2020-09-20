/*
 * Program: Balancing the robot kit 
 * Programer: Tyler Angus 
 * 
 * Purpose: The purpose of this program is to balance the robot kit by
 * using a PID control system. The program takes the raw data from the 
 * mpu 6050 filters it and then uses a PID to create a balanced robot.  
 * 
 * 
 * 
 * 
 */


#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "helper.h"
#include "pins.h"
#include "motor.h"
#include "PID.h"


//#define DEBUG        
//#define Calibrate
#define NoCalibrate
#define SAMPLE_INTERVAL_MS  10       // milliseconds 33.33Hz
#define PLOT

// Initialize Serial, I2C, and mpu6050
void setup(void)
{
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);
  delay(1000);
  setupMPU6050();
  
  #ifdef DEBUG
    Serial.println(F("setup complete"));
  #endif
}



int main(void)
{
  
  SensorData offset, gyroRate, accel;
  AngleData gyroPosition = {0.0, 0.0, 0.0};
  AngleData deltaPosition;                        // change in position
  unsigned long startTime, elapsedTime;



  float presentAngle;
  float desiredAngle = 88.2;//85.45;
  float motorOutput = 0.0;
  
  
  
  int sampleCount = 0;

    // xyz specifies rotation order
  double roll, pitch;                           // roll rotation about x, pitch rotation about y
  double rollF = 0.0, pitchF = 0.0;  // filtered roll and pitch from accel
  double gyro_xf = 0.0, gyro_yf = 0.0, gyro_zf = 0.0; //filtered gyro x y and z 
   
  init();  // Arduino function to initialize timers, pwm, other hardware
  initMotors();
  setup();
  
  #ifdef DEBUG
    Serial.println(F("ready to calibrate"));
  #endif

  #ifdef Calibrate
  calibrateAccelerometer(&offset, 1000);
  calibrateGyro(&offset, 1000);
  #endif

  #ifdef NoCalibrate
  offset.xG = 206;
  offset.yG = -185;
  offset.zG = 113;
  offset.x = -416;
  offset.y = 737;
  offset.z = 2235; 
  #endif

  #ifdef DEBUG
    Serial.println(F("calibration complete"));
    printOffsetValues(&offset);
    delay(1000);
  #endif
  
  startTime = millis();
  
  while(1)
  {
    if( (elapsedTime = (millis() - startTime)) >= SAMPLE_INTERVAL_MS)
    {
      readAccelerometer(&accel);
      readGyro(&gyroRate);
      startTime = millis(); 
      ++sampleCount;


        
      gyroRate.x += offset.xG;       // apply offset
      gyroRate.y += offset.yG;
      gyroRate.z += offset.zG;

      accel.x += offset.x;          // apply offset
      accel.y += offset.y;
      accel.z += offset.z;

      // Freescale, equations 25 and 26, Rotation order Rxyz
      // roll may vary from -pi to +pi, use atan2
      pitch = atan2(accel.y, accel.z) * 180.0/PI;
      // pitch is restricted to -pi/2 to +pi/2, use atan
      roll = atan(-accel.x / sqrt(pow(accel.y,2) + pow(accel.z,2))) * 180.0/PI;
      
      
      // apply low pass filter
      rollF = 0.94 * rollF + 0.06 * roll;
      pitchF = 0.94 * pitchF + 0.06 * pitch;
      

      // scale the data to deg/s and calculate change in position
      deltaPosition.x = (double)gyroRate.x / 131.0 * elapsedTime / 1000.0;
      deltaPosition.y = (double)gyroRate.y / 131.0 * elapsedTime / 1000.0;
      deltaPosition.z = (double)gyroRate.z / 131.0 * elapsedTime / 1000.0;

      gyroPosition.x += deltaPosition.x;
      gyroPosition.y += deltaPosition.y;
      gyroPosition.z += deltaPosition.z;

      //θ[n] = α * (θ[n-1] + θdotgyro[n] * δt )

      //The general form of a high-pass filter equation is y[i] = α y[i-1] + α (x[i] - x[i-1])
      //where y is the filtered output and x[i] - x[i-1] is the change in input.
      //The integration θdotgyro[n] * δt is the gyro displacement, the equivalent to the x[i] - x[i-1] term. 
      //rollF = 0.94 * rollF + 0.06 * roll;

      gyro_xf = (0.94*gyro_xf) + (0.94*(deltaPosition.x));
      gyro_yf = (0.94*gyro_yf) + (0.94*(deltaPosition.y));
      gyro_zf = (0.94*gyro_zf) + (0.94*(deltaPosition.z));
      
      presentAngle = (gyro_xf + pitchF + 90);//using the angle given by the complementary filter and adding 90 degrees.

      #ifdef DEBUG
        Serial.print("presentAngle=");
        Serial.print(presentAngle);
        Serial.print("\t");
      #endif

      motorOutput = PID(desiredAngle, presentAngle); //calls the PID function and sets the output to motorOutput. 

      #ifdef DEBUG
        Serial.print("motorOutput=");
        Serial.println(motorOutput);
      #endif

      #ifdef PLOT
        Serial.print(motorOutput);
        Serial.println("");
      #endif    
        
        
      balanceBot(motorOutput); //balance bot uses motorOutput to know when to drive forward or backword 
       


    }
  }

    
   
  return 0;
  
}
