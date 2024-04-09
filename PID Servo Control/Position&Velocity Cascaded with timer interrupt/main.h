#pragma once
#ifndef main_h
#define main_h

#define ENCA 2 
#define ENCB 3  
#define PWM_PIN 5
#define DIR_PIN 10



// PID constants
 float kp_v = 3.75;
 float ki_v = 34.48   *  3.75;
 float kp_p = 3.75;
 float x= ki_v;


/*long start_time   = 0;

volatile int posi = 0; //current position from encoder
double pos = 0 ;       //current position in thee loop
float  velocity =0;
float eintegral   =0; //integral error

float pos_setpoint=0; //target position
float vel_setpoint=0; //target velocity
double vel_error = 0;

float Pos_ControlSignal = 0;
float Vel_ControlSignal = 0;

bool TIME_5ms=0;*/


extern volatile int posi;
void setMotor(int dir, int pwmVal);

void readEncoder1();

void readEncoder2();

/*void SetPoints(float &pos_setpoint ,float &vel_setpoint);*/

//void Set_Timer();
#endif
