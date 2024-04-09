//#include <TimerOne.h>

#include "main.h"
// PID constants
float kp_v = 3.7;
float ki_v = 40.93 * 3.7 ;
float kp_p = 8;

//PID variables
long    start_time = 0;
volatile int posi = 0; //current position
long        prevT = 0; //previous time

float  eintegral   = 0; //integral error
float  pos_setpoint = 0; //target position
float  vel_setpoint = 0; //target velocity

float  velocity = 0;
double pos     = 0;

double pos_error = 0;
double vel_error = 0;

float Pos_ControlSignal = 0;
float Vel_ControlSignal = 0;

void setup()
{

  //Encoder
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  //motor
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  //interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder2, RISING);

  analogWrite(PWM_PIN, 0);
  posi = 0;

  Serial.begin(9600);
  Serial.println("Enter Position Set Point: ");
  while (!Serial.available());
  pos_setpoint = fabs (   Serial.parseFloat()  );
  //if (pos_setpoint > 80)  pos_setpoint =80; //max allowed position

  while (!Serial.available());
  vel_setpoint = fabs(   Serial.parseFloat() );
  Serial.println("Enter Velocity Set Point:");
  while (!Serial.available());
  vel_setpoint = fabs(   Serial.parseFloat()  );
  if (vel_setpoint > 90) vel_setpoint = 90; //max allowed speed

  start_time = millis();
  prevT = micros();
}

void loop()
{
  if ( millis() - start_time <   10000 ) //(  pos_error / vel_setpoint ) * 1000 +
  {
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
    prevT = currT;
    /* Read  position and velocity*/
    /*interrupt sandwich*/
    noInterrupts();
    velocity = (  (posi / 40.98)   - pos )   / deltaT  ; //(current pos. - last pos.) / time
    pos     = (double(posi) / 40.98);  //102.3 = NO. of encoder pulses for 1 cm.
    interrupts();

    //Position Control Signal
    Pos_ControlSignal = (pos_setpoint - pos ) * kp_p ; //error *Kp

    /*saturation of the control signal with target velocity*/
    if (fabs( Pos_ControlSignal )  > vel_setpoint)
    {
      if (Pos_ControlSignal >= 0)
        Pos_ControlSignal = vel_setpoint;
      else Pos_ControlSignal = - vel_setpoint;
    }

    vel_error = Pos_ControlSignal - velocity; //setpoint - current velocity

    // integral
    eintegral = eintegral + vel_error * deltaT;

    //velocity control signal
    Vel_ControlSignal =  kp_v  *  vel_error + ki_v * eintegral;


    int dir = 1;
    if (Vel_ControlSignal < 0)  dir = -1;

    // motor power
    float pwr = fabs(Vel_ControlSignal);
    if ( pwr >= 255 )
    {
      pwr = 255;
      eintegral -= vel_error * deltaT; //antiWindup
    }
    // signal the motor
    setMotor(dir, pwr);


    Serial.print(velocity);
    Serial.print("   ");
    Serial.print(pos);
    Serial.println();
  }

  else //EXEEDED THE PERIOD
  {
    setMotor(0, 0);
    while (!Serial.available());
    pos_setpoint = fabs (Serial.parseFloat());
    Serial.println ("Enter Position Set Point: ") ;
    while (!Serial.available());
    pos_setpoint = fabs (Serial.parseFloat());
    //if (pos_setpoint > 80) pos_setpoint =80;


    while (!Serial.available());
    vel_setpoint = fabs (Serial.parseFloat()  );
    Serial.println("Enter Velocity Set Point:");
    while (!Serial.available());
    vel_setpoint = fabs (Serial.parseFloat()  );
    if (vel_setpoint > 84)  vel_setpoint = 84;

    eintegral = 0;
    start_time = millis();
    prevT = micros();
  }
}
