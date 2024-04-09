#include "main.h"

// PID constants
 float kp = 65.1;
 float ki = 63;
 float kd = 2.1;
 //float x=ki;
 
//PID variables
long start_time=0;
volatile int posi = 0; //current position 
long prevT = 0;        //previous time
float eprev = 0;       //previous error
float eintegral = 0;   //integral error
float setpoint=0;      //target position

void setup() 
{
  Serial.begin(9600);
  //Encoder 
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  //motor
  pinMode(PWM_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);
  //interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB),readEncoder2,RISING);

 
 /* Serial.println("Enter controllers constant :");
  delay(500);
  
  Serial.println("Enter Kp: ");
  while (!Serial.available());
  kp=Serial.parseFloat();
  Serial.println(kp);
  delay(100);
  

  while (!Serial.available());
  ki=Serial.parseFloat();
  Serial.println("Enter Ki:");
  while (!Serial.available());
  ki=Serial.parseFloat();
  Serial.println(ki);
  delay(100);


  while (!Serial.available());
  kd=Serial.parseFloat();
  Serial.println("Enter Kd: ");
  while (!Serial.available());
  kd=Serial.parseFloat();
  Serial.println(kd);
  delay(100);


  while (!Serial.available());
  setpoint=Serial.parseFloat();*/
  Serial.println("Enter Set Point: ");
  while (!Serial.available());
  setpoint=Serial.parseFloat();


 /* kp= (-0.5  * abs(setpoint) +80) ;
  ki= (-0.05 * abs(setpoint) + 4.45) ;
  kd= (0.01  *abs(setpoint)-0.045) ;*/


  //x=ki;
  start_time=millis();
  prevT = micros();
  
}

void loop()
{

  if ( millis() - start_time < 4000)
  {
  
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  double pos = 0; 
  
  /*interrupt sandwich*/
  noInterrupts(); 
  pos = ( double(posi) / 102.313892 ); //102.3 = NO. of encoder pulses for 1 cm.
  interrupts(); 
  
  // error
  double e = setpoint - pos;
  // derivative
  float dedt = (e-eprev) / (deltaT);
  // integral
  eintegral = eintegral + e*deltaT;
  // control signalt
  float ControlSignal = kp*e + kd*dedt + ki*eintegral;
  // motor direction
  int dir = 1;
  if (ControlSignal < 0)  dir = -1;
  
  
  // motor power
  float pwr = fabs(ControlSignal);
  if( pwr >= 255 )
  {
    pwr = 255;
    //Anti_Windup
     eintegral -= e*deltaT;
  }
 /*  if ( e * ControlSignal > 0)
         ki=0;
        
    else ki=x;  
  }
   else ki = x;*/
   
  // signal the motor
  setMotor(dir,pwr);
  // store previous error
  eprev = e;

  Serial.print(pos);
  Serial.println();
  }

  else
  {
     analogWrite(PWM_PIN,0); 
     delay(500);
     
    //enter new set point
     while (!Serial.available());
     setpoint=Serial.parseFloat();
     Serial.println("Enter Set Point: "); 
     while (!Serial.available());
     setpoint=Serial.parseFloat();
   

 /*    //get new control paramters
     kp= (-0.5  * abs(setpoint) +80)   ; 
     ki= (-0.05 * abs(setpoint) + 4.45);
     kd= (0.01  *abs(setpoint)-0.045)  ;*/

  
     eintegral  = 0;
     eprev      = 0;
     start_time = millis();
     prevT      = micros();
   }

}
