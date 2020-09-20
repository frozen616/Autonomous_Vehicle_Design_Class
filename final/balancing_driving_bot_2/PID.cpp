


#include "PID.h"

#include <Arduino.h>  

//#define DEBUG_BALANCE
//#define DEBUG_MOTOR
//#define PLOT_BALANCE
//#define LEFT_PLOT_MOTOR
//#define RIGHT_PLOT_MOTOR
//#define DEBUG_SPEED
//#define SPEED_PLOT


float PID_balance(float desiredAngle, float presentAngle, float PID_speed_output){
  float error = 0; 
  const float kp = 60.0;//60
  const float kd = 0.1;//0.1
  const float ki = 140.0;//140
  float currentTime = 0.0;
  float dT = 0.0;
  float pid = 0.0;
  float dterm = 0.0;
  

  static float lastTime = 0.0;
  static float iterm = 0.0;

  static float prevAngle = 0.0;

  //read the timer values
  currentTime = (millis()/1000.0); 
  dT = currentTime - lastTime;
  lastTime = currentTime;
  
  #ifdef DEBUG_BALANCE
  Serial.print("Balancing #'s   ");
  Serial.print("currentTime=");
  Serial.print(currentTime);
  Serial.print("\t");
  #endif

  #ifdef PLOT_BALANCE
  Serial.print(presentAngle);
  Serial.print(",");
  #endif


  #ifdef DEBUG_BALANCE
  Serial.print("dT=");
  Serial.print(dT);
  Serial.print(" \t");
  #endif

  


  error = desiredAngle + PID_speed_output - presentAngle;

  #ifdef DEBUG_BALANCE
  Serial.print("error=");
  Serial.print(error);
  Serial.print("\t");
  #endif

  #ifdef PLOT_BALANCE
  Serial.print(error);
  Serial.print(",");
  #endif
  
  //calculate integral term, the cumulative error
  iterm = iterm + error*dT;

  //calculate derivative term, the rate error
  dterm = (presentAngle - prevAngle)/dT;

  //update previous to current
  prevAngle = presentAngle;

  //calculate PID value
  pid = (error*kp) + (iterm*ki) + (dterm*kd);

  #ifdef DEBUG_BALANCE
  Serial.print("pid=");
  Serial.print(pid);
  Serial.println("\t");
  #endif

    #ifdef PLOT_BALANCE
  Serial.print(pid);
  Serial.print(",");
  #endif



  return pid;
  
}








float PID_left_motor(float left_desired_speed, float left_measured_speed){
  float error = 0; 
  const float kp = 3.5;//3.5 had the best results while testing.
  const float kd = 0.1;//0.1 had the best results during testing
  const float ki = 0.0;//adding ki made the results bad
  float currentTime = 0.0;
  float dT = 0.0;
  float pid = 0.0;
  float dterm = 0.0;
  

  static float lastTime = 0.0;
  static float iterm = 0;

  static float prev_speed = 0.0;

  //read the timer values
  currentTime = (millis()/1000.0); 
  dT = currentTime - lastTime;
  lastTime = currentTime;
  
  #ifdef DEBUG_MOTOR
    Serial.println("----------------------------------------------------------------------");
    Serial.println("Left Motor PID numbers");
    Serial.print("currentTime=");
    Serial.print(currentTime);
    Serial.println("");
  #endif

  #ifdef LEFT_PLOT_MOTOR
    Serial.print(left_measured_speed);
    Serial.print(",");
  #endif


  #ifdef DEBUG_MOTOR
    Serial.print("dT=");
    Serial.print(dT);
    Serial.print(" \t");
  #endif

  


  error = left_desired_speed - left_measured_speed;

  #ifdef DEBUG_MOTOR
    Serial.print("error=");
    Serial.print(error);
    Serial.print("\t");
  #endif

  #ifdef LEFT_PLOT_MOTOR
    Serial.print(error);
    Serial.print(",");
  #endif
  
  //calculate integral term, the cumulative error
  iterm = iterm + error*dT;

  //calculate derivative term, the rate error
  dterm = (left_measured_speed - prev_speed)/dT;

  //update previous to current
  prev_speed = left_measured_speed;

  //calculate PID value
  pid = (error*kp) + (iterm*ki) + (dterm*kd);

  #ifdef DEBUG_MOTOR
    Serial.print("pid=");
    Serial.print(pid);
    Serial.println("\t");
    Serial.println("----------------------------------------------------------------------");
  #endif

   #ifdef LEFT_PLOT_MOTOR
    Serial.println(pid);
  
  #endif

  //limit the pid to 255 the max speed 
  if ( pid > 255 ){
    pid = 255;
  }
  else if (pid < -255){
    pid = -255;
  }

  return pid;
  
}









float PID_right_motor(float right_desired_speed, float right_measured_speed){
  float error = 0; 
  const float kp = 2.5;//2.5 had the best results during testing
  const float kd = 0.1;//0.1 had the best results during testing
  const float ki = 0.0;//adding ki caused the system to be off.
  float currentTime = 0.0;
  float dT = 0.0;
  float pid = 0.0;
  float dterm = 0.0;
  

  static float lastTime = 0.0;
  static float iterm = 0;

  static float prev_speed = 0.0;

  //read the timer values
  currentTime = (millis()/1000.0); 
  dT = currentTime - lastTime;
  lastTime = currentTime;
  
  #ifdef DEBUG_MOTOR
    Serial.println("----------------------------------------------------------------------");
    Serial.println("Right Motor PID numbers");
    Serial.print("currentTime=");
    Serial.print(currentTime);
    Serial.println("");
  #endif

  #ifdef RIGHT_PLOT_MOTOR
    Serial.print(right_measured_speed);
    Serial.print(",");
  #endif


  #ifdef DEBUG_MOTOR
    Serial.print("dT=");
    Serial.print(dT);
    Serial.print(" \t");
  #endif

  


  error = right_desired_speed - right_measured_speed;

  #ifdef DEBUG_MOTOR
    Serial.print("error=");
    Serial.print(error);
    Serial.print("\t");
  #endif

  #ifdef RIGHT_PLOT_MOTOR
    Serial.print(error);
    Serial.print(",");
  #endif
  
  //calculate integral term, the cumulative error
  iterm = iterm + error*dT;

  //calculate derivative term, the rate error
  dterm = (right_measured_speed - prev_speed)/dT;

  //update previous to current
  prev_speed = right_measured_speed;

  //calculate PID value
  pid = (error*kp) + (iterm*ki) + (dterm*kd);

  #ifdef DEBUG_MOTOR
    Serial.print("pid=");
    Serial.print(pid);
    Serial.println("\t");
    Serial.println("----------------------------------------------------------------------");
  #endif

   #ifdef RIGHT_PLOT_MOTOR
    Serial.println(pid);
  
  #endif

  //limit the pid to 255 the max speed 
  if ( pid > 255 ){
    pid = 255;
  }
  else if (pid < -255){
    pid = -255;
  }

  return pid;
  
}













float PID_speed(float desired_speed, float mean_speed){
  float error = 0; 
  const float kp = 1.0;
  const float kd = 0.0;
  const float ki = 0.0;
  float currentTime = 0.0;
  float dT = 0.0;
  float pid = 0.0;
  float dterm = 0.0;
  

  static float lastTime = 0.0;
  static float iterm = 0;

  static float prev_speed = 0.0;

  //read the timer values
  currentTime = (millis()/1000.0); 
  dT = currentTime - lastTime;
  lastTime = currentTime;
  
  #ifdef DEBUG_SPEED
    Serial.println("----------------------------------------------------------------------");
    Serial.println("Speed PID #'s");
    Serial.print("currentTime=");
    Serial.print(currentTime);
    Serial.println("");
  #endif

  #ifdef SPEED_PLOT
    Serial.print(mean_speed);
    Serial.print(",");
  #endif


  #ifdef DEBUG_SPEED
    Serial.print("dT=");
    Serial.print(dT);
    Serial.print(" \t");
  #endif

  


  error = desired_speed - mean_speed;

  #ifdef DEBUG_SPEED
    Serial.print("error=");
    Serial.print(error);
    Serial.print("\t");
  #endif

  #ifdef SPEED_PLOT
    Serial.print(error);
    Serial.print(",");
  #endif
  
  //calculate integral term, the cumulative error
  iterm = iterm + error*dT;

  //calculate derivative term, the rate error
  dterm = (mean_speed - prev_speed)/dT;

  //update previous to current
  prev_speed = mean_speed;

  //calculate PID value
  pid = (error*kp) + (iterm*ki) + (dterm*kd);

  #ifdef DEBUG_SPEED
    Serial.print("pid=");
    Serial.print(pid);
    Serial.println("\t");
    Serial.println("----------------------------------------------------------------------");
  #endif

   #ifdef SPEED_PLOT
    Serial.println(pid);
  
  #endif

 

  return pid;
  
}
