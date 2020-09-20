#ifndef PID_H_MOTOR_CONTROL
#define PID_H_MOTOR_CONTROL

float PID_balance(float desiredAngle, float presentAngle, float PID_speed_output);

float PID_left_motor(float left_desired_speed, float left_measured_speed);

float PID_right_motor(float right_desired_speed, float right_measured_speed);

float PID_speed(float desired_speed, float mean_speed);


#endif
