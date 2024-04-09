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
    posi--;
  
   else
    posi++;
}

void readEncoder2()
{
   int b = digitalRead(ENCA);
   if(b > 0)
    posi++;
    
   else
    posi--;
}
