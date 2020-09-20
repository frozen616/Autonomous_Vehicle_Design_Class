#include "pins.h"
#include "motor.h"
#include <Arduino.h>

#define DEBUG

void initMotors( void)
{
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(PWMA_LEFT_PIN, OUTPUT);
  pinMode(PWMB_RIGHT_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  stopMotors();
}

void stopMotors(void)
{
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT_PIN, 0);
  analogWrite(PWMB_RIGHT_PIN, 0);
}

void stop_right_motor(void)
{
  digitalWrite(BIN1_PIN, LOW);
  analogWrite(PWMB_RIGHT_PIN, 0);
}

void stop_left_motor(void)
{
  digitalWrite(AIN1_PIN, LOW);
  analogWrite(PWMA_LEFT_PIN, 0);
}

void drive_left_forward(unsigned char left_PID_out){
  digitalWrite(AIN1_PIN, 0);
  analogWrite(PWMA_LEFT_PIN, left_PID_out); 
}

void drive_right_forward(unsigned char right_PID_out){
  digitalWrite(BIN1_PIN, 0);
  analogWrite(PWMB_RIGHT_PIN, right_PID_out);
}

void drive_left_backward(unsigned char left_PID_out){
  digitalWrite(AIN1_PIN, 1);
  analogWrite(PWMA_LEFT_PIN, left_PID_out); 
}

void drive_right_backward(unsigned char right_PID_out){
  digitalWrite(BIN1_PIN, 1);
  analogWrite(PWMB_RIGHT_PIN, right_PID_out);
}

void driveForward(unsigned char speed)
{
  digitalWrite(AIN1_PIN, 0);
  digitalWrite(BIN1_PIN, 0);
  analogWrite(PWMA_LEFT_PIN, speed);
  analogWrite(PWMB_RIGHT_PIN, speed);
}


void driveBackward(unsigned char speed)
{
  digitalWrite(AIN1_PIN, 1);
  digitalWrite(BIN1_PIN, 1);
  analogWrite(PWMA_LEFT_PIN, speed);
  analogWrite(PWMB_RIGHT_PIN, speed);
}


void balanceBot(float left_PID_out, float right_PID_out){
  
  if(left_PID_out>0) {
    drive_left_backward(char(abs(left_PID_out)));
  }
  if(left_PID_out==0){
    stop_left_motor();
  }
  if(left_PID_out<0) {
    drive_left_forward(char(abs(left_PID_out)));
  }

  if(right_PID_out>0) {
    drive_right_backward(char(abs(right_PID_out)));
  }
  if(right_PID_out==0){
    stop_right_motor();
  }
  if(right_PID_out<0) {
    drive_right_forward(char(abs(right_PID_out)));
  }

  
}
