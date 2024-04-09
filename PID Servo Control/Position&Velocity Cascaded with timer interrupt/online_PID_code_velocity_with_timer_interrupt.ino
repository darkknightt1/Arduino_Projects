#include "main.h"
long start_time   = 0;

volatile int posi = 0; //current position from encoder
double pos = 0 ;       //current position in thee loop
float  velocity =0;
float eintegral   =0; //integral error

float pos_setpoint=0; //target position
float vel_setpoint=0; //target velocity
double vel_error = 0;

float Pos_ControlSignal = 0;
float Vel_ControlSignal = 0;

bool TIME_5ms=0;
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
  
  noInterrupts();
  //SetPoints(pos_setpoint  ,  vel_setpoint); //input setpoints from user
  Serial.println("Enter Position Set Point: ");
  while (!Serial.available());
  pos_setpoint=Serial.parseFloat();
  delay(100);

  while (!Serial.available());
  vel_setpoint=Serial.parseFloat();
  Serial.println("Enter Velocity Set Point:");
  while (!Serial.available());
  vel_setpoint=Serial.parseFloat();
 // Set_Timer ();
     //Timer interuppt intialization
     TCNT2 = 0; // Reset the timer
     TCCR2A = 0b00000010;  // CTC Mode
     TCCR2B = 0b00000111;  // Prescaler 1024
     OCR2A = 78;  // Compare match every 4.992 ms
     TIMSK2 = 0b00000010;  // Enable INT on compare match A
     sei();  // Enable the global interrupt
  interrupts();
  
  start_time=millis();
}

void loop()
{
  if (millis() - start_time < 5000)
  {
    if (TIME_5ms)
    {
    // Read the position
    /*interrupt sandwich*/
    noInterrupts(); 
    velocity=(  (posi / 102.313892)   - pos)   / 0.004992 ;
    pos     =(   double(posi) / 102.313892 ); //102.3 = NO. of encoder pulses for 1 cm.
    interrupts(); 
  

    //Position Control Signal
    Pos_ControlSignal = ( pos_setpoint - pos ) * kp_p ; //error *Kp
    
    //saturation of control signal with velocity set poiint 
    if (fabs( Pos_ControlSignal )  > vel_setpoint)  
    {
       if (Pos_ControlSignal >= 0)
       Pos_ControlSignal = vel_setpoint;
       else Pos_ControlSignal= - vel_setpoint; 
    }
    
    vel_error = Pos_ControlSignal - velocity; //setpoint - current velocity

   // integral
   eintegral = eintegral + vel_error * 0.004992;
 
   //velocity control signal
   Vel_ControlSignal =  kp_v  *  vel_error + ki_v * eintegral;

   int dir = 1;
   if (Vel_ControlSignal < 0)  dir = -1;
 
   // motor power
   float pwr = fabs(Vel_ControlSignal);
   if( pwr >= 255 )
   {
    pwr = 255;
    //Anti_Windup
    if ( vel_error * Vel_ControlSignal > 0)  ki_v=0; 
    else ki_v = x;
   }
   else  ki_v = x;

  // signal the motor
  setMotor(dir,pwr);

    Serial.print(velocity);
    Serial.print("\t");
    Serial.print(pos);
    Serial.println();
    TIME_5ms=0;
   }
  }
  
  else setMotor(0,0);
 }


 ISR(TIMER2_COMPA_vect)//timer interrupt
{
  TIME_5ms = 1;
}
