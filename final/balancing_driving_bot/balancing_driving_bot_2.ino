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
#include "voltage.h"
#include <util/atomic.h>

//#define DEBUG 
#define DEBUG_BALANCE      
//#define Calibrate
#define NoCalibrate
#define SAMPLE_INTERVAL_MS  5       // 5 miliseconds 200hz
//#define PLOT
//#define PLOT_PID_MOTOR
//#define DEBUG_OUTPUT

// Global Constants
const unsigned long ENCODER_SAMPLE_INTERVAL = 100UL;    // units, milliseconds
float speed_calc = 1000.000/float(ENCODER_SAMPLE_INTERVAL);

// Global Variables
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;


/* Interrupt Service Routine
 *  updates left encoder count
 */
void leftEncoderISR(void)
{
  leftEncoderCount++;
}

ISR(PCINT2_vect)
{
  rightEncoderCount++;
}

// Initialize Serial, I2C, and mpu6050
void setup(void)
{
  //set up interupt for left encoder
  pinMode(LEFT_ENCODER_A_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), leftEncoderISR, CHANGE);

  
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);
  setupMPU6050();
  voltageInit();
  // setup for a pin change interupt 
  cli();//turn off interupts 
  PCICR |= 0b00000100; //enables port D pin change interrupts 
  PCMSK2 |= 0b00010000; //PCINT20
  sei();//enable interupts 
  delay(4000);
  
  #ifdef DEBUG
    Serial.println(F("setup complete"));
  #endif
}



int main(void)
{
  
  SensorData offset, gyroRate, accel;
  AngleData gyroPosition = {0.0, 0.0, 0.0};
  unsigned long startTime, startTimeEncoder, dTEncoder, elapsedTime;
  AngleData deltaPosition;                        // change in position
  int count = 0;
  unsigned long encoder_count_left[32] = {0};
  unsigned long encoder_count_right[32] = {0};
  unsigned long elapsed_time[32] = {0};
  float battery_voltage[32] = {0};
  int increment_count[32] = {0};

  float left_PID_out = 0.0;
  float right_PID_out = 0.0;
  float left_desired_speed = 127.0;
  float right_desired_speed = 127.0;
  float left_measured_speed = 0.0;
  float right_measured_speed = 0.0;

  float mean_speed = 0.0;
  float desired_speed = 0.0;
  float PID_speed_output;



  


  float presentAngle;
  float desiredAngle = 0.0;//85.45;
  float motorOutput = 0.0;

  


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
  startTimeEncoder = millis();


  
  
  while(1)
  {


    
    if( (elapsedTime = (millis() - startTime)) >= SAMPLE_INTERVAL_MS)
    {
      readAccelerometer(&accel);
      readGyro(&gyroRate);
      startTime = millis(); 



        
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
      
      presentAngle = (gyro_xf + pitchF);//using the angle given by the complementary filter and adding 90 degrees.

      #ifdef DEBUG_BALANCE
        Serial.print("presentAngle=");
        Serial.print(presentAngle);
        Serial.print("\t");
      #endif
      
      PID_speed_output = PID_speed(desired_speed, mean_speed);

      motorOutput = PID_balance(desiredAngle, presentAngle, PID_speed_output); //calls the PID function and sets the output to motorOutput. 

      left_desired_speed = motorOutput;
      right_desired_speed = motorOutput;
                
      left_PID_out = PID_left_motor(left_desired_speed, left_measured_speed);
      right_PID_out = PID_right_motor(right_desired_speed, right_measured_speed);
      
      balanceBot(left_PID_out, right_PID_out); //balance bot uses motorOutput to know when to drive forward or backword

        
       ///The below if statements take care of the speed measurement if the wheels are going backwards.
        if(left_PID_out<0) {
          left_measured_speed = left_measured_speed * -1;
        }

        if(right_PID_out>0) {
          
        }
       
        if(right_PID_out<0) {
          right_measured_speed = right_measured_speed * -1;
        }

      #ifdef DEBUG_BALANCE
        Serial.print("motorOutput=");
        Serial.println(motorOutput);
      #endif

      #ifdef PLOT
        Serial.print(motorOutput);
        Serial.println("");
      #endif    
        
      
      
       


    }

    if( (millis() - startTimeEncoder) >=  ENCODER_SAMPLE_INTERVAL)
      {
        //keeps the time and stores it in the array
        dTEncoder = millis() - startTimeEncoder;
        elapsed_time[count] = {dTEncoder};        
        startTimeEncoder = millis();
      
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          // code with interrupts blocked (consecutive atomic operations will not get interrupted)

          //set the counts and voltage into the arrays created 
          increment_count[count] = count; 
          encoder_count_left[count] = leftEncoderCount;
          encoder_count_right[count] = rightEncoderCount;
          battery_voltage[count] = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
          left_measured_speed = ((float(encoder_count_left[count])/782.000)*speed_calc*0.210);//this converts the count into velocity the .21 is circumference of the wheel
          right_measured_speed = ((float(encoder_count_right[count])/782.000)*speed_calc*0.210);//right motor velocity conversion
          

           }

          

          #ifdef DEBUG
              Serial.print("increment_count=");
              Serial.print(increment_count[count]);              
              Serial.print("  elapsed time[ms]=");              
              Serial.println(elapsed_time[count]);              
              Serial.print("  encoder count left=");
              Serial.print(encoder_count_left[count]);              
              Serial.print("  encoder count right=");
              Serial.print(encoder_count_right[count]);        
              Serial.print("  Voltage=");
              Serial.println(battery_voltage[count]);
              Serial.print("velocity of left motor = ");
              Serial.print(left_measured_speed, 4);              
              Serial.print("\tvelocity of right motor = ");
              Serial.println(right_measured_speed);
              Serial.print("distance traveled left motor = ");
              Serial.print((encoder_count_left[count]/782.0)*0.21);              
              Serial.print("\tdistance traveled right motor = ");
              Serial.println((encoder_count_right[count]/782.0)*0.21);
              
          #endif 
          
          
          left_measured_speed = (left_measured_speed / 0.70) * (255.0); //.7m/s is the max velocity of the left motor and 255 is the max char that can be sent to the motor. 
          right_measured_speed = (right_measured_speed / 0.71) * (255.0);//.71m/s is the max velocity of the right motor during testing

          

          #ifdef DEBUG
            Serial.print("left_measured_speed = ");
            Serial.print(left_measured_speed, 6); //prints the number with 6 decimal points
            Serial.print("    right_measured_speed = ");
            Serial.println(right_measured_speed, 6);
          #endif
          
          mean_speed = ((left_measured_speed + right_measured_speed)/ 2);

          
          #ifdef DEBUG_OUTPUT
            Serial.print(left_PID_out);
            Serial.println(right_PID_out);
          #endif
          
         
          balanceBot(left_PID_out, right_PID_out);  
            
          #ifdef DEBUG
            Serial.println("=============================================================");
          #endif

          count++;
          leftEncoderCount = 0UL;       // 0 is type int, 0UL is type unsigned long
          rightEncoderCount = 0UL;

      }
 
      
  }
  return 0;
  

  
}
