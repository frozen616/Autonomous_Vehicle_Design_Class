/*  
 *   Homework 4
 *   Programmer: Tyler Angus
 *   
 *   Tilt sensing of roll, pitch, yaw angle from gryo data and pitch and roll from the accelerometer
 *   
 *   
 *  Setup
 *    MPU-6050 - default configuration 
 *    i2c clock frequency 400000
 *    Serial baud rate 115200
 *  
 *  
 *  
 */
#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "helper.h"


//#define DEBUG                     // turn on for all debug info
//#define PRINT_ALL_GYRO              // prints all gyro related info
//#define PRINT_POSITION              // prints gyro position estimate, summed over time  
#define PLOT_POSITION             // turn on for serial plotter
#define PLOT_ROLL            // turn on for serial plotter
#define PLOT_PITCH

#define SAMPLE_INTERVAL_MS  30           // milliseconds


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
  unsigned long startTime, elapsedTimems;

  int sampleCount = 0;

    // xyz specifies rotation order
  double roll, pitch;                           // roll rotation about x, pitch rotation about y
  double rollF = 0.0, pitchF = 0.0;  // filtered roll and pitch from accel
  double gyro_xf = 0.0, gyro_yf = 0.0, gyro_zf = 0.0; //filtered gyro x y and z 
   
  init();                           // Arduino function to initialize timers, pwm, other hardware
  setup();
  
  #ifdef DEBUG
    Serial.println(F("ready to calibrate"));
  #endif

  calibrateAccelerometer(&offset, 10);
  calibrateGyro(&offset, 10);

  #ifdef DEBUG
    Serial.println(F("calibration complete"));
    printOffsetValues(&offset);
    delay(1000);
  #endif
  
  startTime = millis();
  while(1)
  {
    if( (elapsedTimems = (millis() - startTime)) >= SAMPLE_INTERVAL_MS)
    {
      readAccelerometer(&accel);
      readGyro(&gyroRate);
      startTime = millis(); 
      ++sampleCount;

      #ifdef DEBUG
        Serial.println("before offset");
        printSensorData(&gyroRate);
      #endif
        
      gyroRate.x += offset.x;       // apply offset
      gyroRate.y += offset.y;
      gyroRate.z += offset.z;

      accel.x += offset.x;          // apply offset
      accel.y += offset.y;
      accel.z += offset.z;

      // Freescale, equations 25 and 26, Rotation order Rxyz
      // roll may vary from -pi to +pi, use atan2
      pitch = atan2(accel.y, accel.z) * 180.0/PI;
      // pitch is restricted to -pi/2 to +pi/2, use atan
      roll = atan(-accel.x / sqrt(pow(accel.y,2) + pow(accel.z,2))) * 180.0/PI;
      
      #ifdef DEBUG
        Serial.println("Rotation xyz");
        printAngles(roll, pitch);
        Serial.println("");
      #endif
           
      #ifdef DEBUG
        Serial.println("after offset");
        printSensorData(&gyroRate);
      #endif

      // apply low pass filter
      rollF = 0.94 * rollF + 0.06 * roll;
      pitchF = 0.94 * pitchF + 0.06 * pitch;
      
      #ifdef PLOT_ROLL
        
        //Serial.print(roll);
        //Serial.print(",");
        Serial.print(rollF);
      #endif

      #ifdef PLOT_PITCH
        
        //Serial.print(pitch);
        Serial.print(",");
        Serial.print(pitchF);
      #endif

      #ifdef PRINT_ALL_GYRO
        Serial.println("gyro rate, raw data");
        printSensorData(&gyroRate);
      #endif

      // scale the data to deg/s and calculate change in position
      deltaPosition.x = (double)gyroRate.x / 131.0 * elapsedTimems / 1000.0;
      deltaPosition.y = (double)gyroRate.y / 131.0 * elapsedTimems / 1000.0;
      deltaPosition.z = (double)gyroRate.z / 131.0 * elapsedTimems / 1000.0;

      #ifdef PRINT_ALL_GYRO
        Serial.println("change in position, deg");
        printAngleData(&deltaPosition, 4);
      #endif

      gyroPosition.x += deltaPosition.x;
      gyroPosition.y += deltaPosition.y;
      gyroPosition.z += deltaPosition.z;


      #ifdef PRINT_ALL_GYRO
        Serial.println("gyro position, deg");
        printAngleData(&gyroPosition,4);
      #endif
     
      #ifdef PRINT_POSITION
        if( sampleCount % 100 == 0 )
        {
          Serial.println("gyro position, deg");
          printAngleData(&gyroPosition,4);
          sampleCount = 0;
        }
      #endif
            //θ[n] = α * (θ[n-1] + θdotgyro[n] * δt )

//The general form of a high-pass filter equation is y[i] = α y[i-1] + α (x[i] - x[i-1])
//where y is the filtered output and x[i] - x[i-1] is the change in input.
//The integration θdotgyro[n] * δt is the gyro displacement, the equivalent to the x[i] - x[i-1] term. 
//rollF = 0.94 * rollF + 0.06 * roll;

       gyro_xf = (0.94*gyro_xf) + (0.94*(deltaPosition.x));
       gyro_yf = (0.94*gyro_yf) + (0.94*(deltaPosition.y));
       gyro_zf = (0.94*gyro_zf) + (0.94*(deltaPosition.z));
       
      #ifdef PLOT_POSITION
        
        Serial.print(",");
        Serial.print(gyroPosition.x);
        Serial.print(",");
        Serial.print(gyroPosition.y);
        Serial.print(",");
        Serial.print(gyroPosition.z);
        Serial.print(",");
        //add up the high pass and low pass filtered data for pitch and roll 
        //creates the complementary filter
        Serial.print(gyro_xf + pitchF);
        Serial.print(",");
        Serial.println(gyro_yf + rollF);
      #endif


    }
  }
    
   
  return 0;
  
}
