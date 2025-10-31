#ifndef MOTORDC_h
#define MOTORDC_h

#include <Arduino.h>

volatile int n = 0;
volatile byte ant = 0;
volatile byte act = 0;

double N = 0.0;             
int pwmMotor = 29; // PWM inicial mínimo para arrancar motor: CALIBRAR

unsigned long lastTime = 0;
const int R = 4704; 

// PID  
double input, pid_Motor;      

double integral_M = 0, derivative_M = 0, lastError = 0;

void computeRpm(void){
  unsigned long now = millis();
  unsigned long elapsed = now - lastTime;
  if (elapsed > 0) {
    N = (n * 60.0 * 1000.0) / (elapsed * R);
  } else {
    N = 0;
  }
  lastTime = now;
  n = 0;
} 

void encoder(void){
  ant = act;                         
  act = PIND & 12;             
                         
  if(ant==0  && act== 4)  n++;
  if(ant==4  && act==12)  n++;
  if(ant==8  && act== 0)  n++;
  if(ant==12 && act== 8)  n++;
 
  if(ant==0 && act==8)  n--; 
  if(ant==4 && act==0)  n--;
  if(ant==8 && act==12) n--;
  if(ant==12 && act==4) n--;   
}

double PIDMotor(float dt, double setpoint_Motor, double Kp_M, double Ki_M, double Kd_M) {
  //PID compute for the motor
  double error_Motor = setpoint_Motor - input;
  integral_M += error_Motor * dt;  
  derivative_M = (error_Motor - lastError) / dt;

  pid_Motor = Kp_M * error_Motor + Ki_M * integral_M + Kd_M * derivative_M;

  if (pid_Motor > 255) {
    pid_Motor = 255;
    integral_M -= error_Motor * dt;
  }
  if (pid_Motor < 0) {
    pid_Motor = 0;
    integral_M -= error_Motor * dt;
  }

  if (pid_Motor > 0 && pid_Motor < 30) pid_Motor = 30;
  lastError = error_Motor;
  return pid_Motor;
}

#endif