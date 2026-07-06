#ifndef MOTORDC_h
#define MOTORDC_h
#include <Arduino.h>

// ENCODER AND SPEED STATE
volatile int n = 0;
volatile byte prev = 0;
volatile byte curr = 0;
double N = 0.0;
int pwmMotor = 29;             // Minimum PWM to start the motor: CALIBRATE
unsigned long lastTime = 0;
const int R = 4704;

// PID STATE
double input, pid_Motor;
double integral_M = 0, derivative_M = 0, lastError = 0;

// RPM COMPUTATION FROM ENCODER
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

// QUADRATURE ENCODER ISR
void encoder(void){
  prev = curr;
  curr = PIND & 12;

  if(prev==0  && curr== 4)  n++;
  if(prev==4  && curr==12)  n++;
  if(prev==8  && curr== 0)  n++;
  if(prev==12 && curr== 8)  n++;

  if(prev==0 && curr==8)  n--;
  if(prev==4 && curr==0)  n--;
  if(prev==8 && curr==12) n--;
  if(prev==12 && curr==4) n--;
}

// DC MOTOR PID
double PIDMotor(float dt, double setpoint_Motor, double Kp_M, double Ki_M, double Kd_M) {
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
