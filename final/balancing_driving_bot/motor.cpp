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

void drive_left_forward(unsigned char left_PID_out){
  digitalWrite(AIN1_PIN, 0);
  analogWrite(PWMA_LEFT_PIN, left_PID_out); 
}

void drive_right_forward(unsigned char right_motor_speed){
  digitalWrite(BIN1_PIN, 0);
  analogWrite(PWMB_RIGHT_PIN, right_motor_speed);
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


void balanceBot(float motorOutput){
  
  if(motorOutput>0) {
    driveBackward(char(abs(motorOutput)));
  }
  if(motorOutput==0){
    stopMotors();
  }
  if(motorOutput<0) {
    driveForward(char(abs(motorOutput)));
  }
}
