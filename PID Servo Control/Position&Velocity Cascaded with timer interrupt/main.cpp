#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "main.h"
void setMotor(int dir, int pwmVal)
{
  analogWrite(PWM_PIN,pwmVal);
  if(dir == 1)
    digitalWrite(DIR_PIN,LOW);
  
  else if(dir == -1)
    digitalWrite(DIR_PIN,HIGH);
  

  
}

void readEncoder1()
{
   int b = digitalRead(ENCB);
   if(b > 0)
    posi++;
  
   else
    posi--;
}

void readEncoder2()
{
   int b = digitalRead(ENCA);
   if(b > 0)
    posi--;
    
   else
    posi++;
}

/*void SetPoints(float &pos_setpoint ,float &vel_setpoint)
{
  Serial.println("Enter Position Set Point: ");
  while (!Serial.available());
  pos_setpoint=Serial.parseFloat();
  delay(100);

  while (!Serial.available());
  vel_setpoint=Serial.parseFloat();
  Serial.println("Enter Velocity Set Point:");
  while (!Serial.available());
  vel_setpoint=Serial.parseFloat();
  
 }*/


 
/*void Set_Timer()
{
    //Timer interuppt intialization
     TCNT2 = 0; // Reset the timer
     TCCR2A = 0b00000010;  // CTC Mode
     TCCR2B = 0b00000111;  // Prescaler 1024
     OCR2A = 78;  // Compare match every 4.992 ms
     TIMSK2 = 0b00000010;  // Enable INT on compare match A
     sei();  // Enable the global interrupt
}*/
