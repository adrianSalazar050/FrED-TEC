#ifndef HOTEND_h
#define HOTEND_h
#include <Arduino.h>

// THERMISTOR CALIBRATION
const float Raux = 460;
const float A = 1.1384e-03, B = 2.3245e-04, C = 9.489e-08;   // Steinhart-Hart constants

// PID STATE
float integral_H = 0, derivative_H = 0, prevError = 0;
unsigned long prevTime;
float pidMin = 0.0, pidMax = 255.0;                          // Direct PWM range
double pid_Hotend;

// SAFETY LIMITS
float maxTemp = 250.0;
float minTemp = 0.0;

// TEMPERATURE FROM THERMISTOR READING
float thermistor(int reading) {
    if (reading <= 10 || reading >= 1020) {
        return -999;                                         // Error: thermistor disconnected
    }
    float R = Raux * ((float)reading / (1023.0 - (float)reading));
    float logR = log(R);
    float TempK = 1.0 / (A + B * logR + C * logR * logR * logR);
    return TempK - 273.15;
}

// HOTEND PID
double PIDHotend(float temp, float dt, double setpoint_Hotend, double Kp_H, double Ki_H, double Kd_H){
  double error_Hotend = setpoint_Hotend - temp;
  integral_H += error_Hotend * dt;
  derivative_H = (error_Hotend - prevError) / dt;
  pid_Hotend = Kp_H * error_Hotend + Ki_H * integral_H + Kd_H * derivative_H;
  prevError = error_Hotend;

  // Integral anti-windup
  if (pid_Hotend > pidMax) {
      pid_Hotend = pidMax;
      integral_H -= error_Hotend * dt;
  } else if (pid_Hotend < pidMin) {
      pid_Hotend = pidMin;
      integral_H -= error_Hotend * dt;
  }
  integral_H = constrain(integral_H, -50, 50);               // Hard clamp against extreme windup
  return pid_Hotend;
}

#endif
