#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED


void initMotors( void);
void stopMotors(void);

void drive_left_forward(unsigned char left_motor_speed);
void drive_right_forward(unsigned char right_motor_speed);
void driveForward(unsigned char speed);
void driveBackward(unsigned char speed);

void drive_right_backward(unsigned char right_PID_out);
void drive_left_backward(unsigned char left_PID_out);
void drive_right_forward(unsigned char right_PID_out);
void drive_left_forward(unsigned char left_PID_out);


void balanceBot(float left_PID_out, float right_PID_out);




#endif 
