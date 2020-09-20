


#include "PID.h"

#include <Arduino.h>  

//#define DEBUG
#define PLOT


float PID(float desiredAngle, float presentAngle){
  float error = 0; 
  float kp = 60.0;//60
  float kd = 0.1;//0.1
  float ki = 140.00;//140
  float currentTime = 0.0;
  float dT = 0.0;
  float pid = 0.0;
  float dterm = 0.0;
  

  static float lastTime = 0.0;
  static float iterm = 0;

  static float prevAngle = 90;

  //read the timer values
  currentTime = (millis()/1000.0); 
  dT = currentTime - lastTime;
  lastTime = currentTime;
  
  #ifdef DEBUG
  Serial.print("currentTime=");
  Serial.print(currentTime);
  Serial.print("\t");
  #endif

  #ifdef PLOT
  Serial.print(presentAngle);
  Serial.print(",");
  #endif


  #ifdef DEBUG
  Serial.print("dT=");
  Serial.print(dT);
  Serial.print(" \t");
  #endif

  


  error = desiredAngle - presentAngle;

  #ifdef DEBUG
  Serial.print("error=");
  Serial.print(error);
  Serial.print("\t");
  #endif

  #ifdef PLOT
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

  #ifdef DEBUG
  Serial.print("pid=");
  Serial.print(pid);
  Serial.print("\t");
  #endif

    #ifdef PLOT
  Serial.print(pid);
  Serial.print(",");
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
