template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order + 1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order + 1]; // Raw values
    float y[order + 1]; // Filtered values

  public:
    LowPass(float f0, float fs, bool adaptive) {
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.

      omega0 = 6.28318530718 * f0;
      dt = 1.0 / fs;
      adapt = adaptive;
      tn1 = -dt;
      for (int k = 0; k < order + 1; k++) {
        x[k] = 0;
        y[k] = 0;
      }
      setCoef();
    }

    void setCoef() {
      if (adapt) {
        float t = micros() / 1.0e6;
        dt = t - tn1;
        tn1 = t;
      }

      float alpha = omega0 * dt;
      if (order == 1) {
        a[0] = -(alpha - 2.0) / (alpha + 2.0);
        b[0] = alpha / (alpha + 2.0);
        b[1] = alpha / (alpha + 2.0);
      }
      if (order == 2) {
        float alphaSq = alpha * alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
        b[0] = alphaSq / D;
        b[1] = 2 * b[0];
        b[2] = b[0];
        a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
        a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
      }
    }

    float filt(float xn) {
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if (adapt) {
        setCoef(); // Update coefficients if necessary
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for (int k = 0; k < order; k++) {
        y[0] += a[k] * y[k + 1] + b[k] * x[k];
      }
      y[0] += b[order] * x[order];

      // Save the historical values
      for (int k = order; k > 0; k--) {
        y[k] = y[k - 1];
        x[k] = x[k - 1];
      }

      // Return the filtered value
      return y[0];
    }
};

// Filter instance
LowPass<2> lp(3, 1e3, true);


#include <Servo.h>
Servo servoMotor;

int emgMin = 300;
int emgMax = 400;
int servoMin = 0;
int servoMax = 100;
int lastposition = 0;
float prvfilteredValue = 1;
long currtime=0;
int arr[100];
int ijk = 0;
int flag=0;
int count = 0;

const int emgPin = A0;  // Analog pin for EMG sensor
const int numSamples = 100;  // Number of samples for RMS calculation

int emgValues[numSamples];  // Array to store EMG samples


const float alpha = 0.04;

float filteredValue = 0.0;

void setup() {
   Serial.begin(9600);
  servoMotor.attach(9);
  //servoMotor.write(70);
  //delay(1000);
  //servoMotor.write(130);
  //delay(1000);
}

void loop() {
  // Read EMG sensor values and store in array
  for (int i = 0; i < numSamples; i++)
    emgValues[i] = lp.filt(analogRead(emgPin));



  double sumSquares = 0;
  for (int i = 0; i < numSamples; i++) {
    sumSquares += pow(emgValues[i], 2);
  }

  double rmsValue = sqrt(sumSquares / numSamples);
  filteredValue  = alpha * rmsValue + (1 - alpha) * filteredValue;

  Serial.println("");
  Serial.print(filteredValue);
if(millis()-currtime>700)
{
  //count++;
 if (prvfilteredValue * 1.1 <= filteredValue )
  {
    if (flag == 1)//&& count==5
    {
      servoMotor.write(0);
      delay(15);
      flag = 0;
      //servoMotor.write(70);
       count = 0;
    }
    else if (flag == 0)//&& count==5
    {
      servoMotor.write(100);
      delay(15);
      flag = 1;
      //servoMotor.wr
       count = 0;
    }  
    

}
prvfilteredValue = filteredValue;
  currtime=millis();
  }
}
