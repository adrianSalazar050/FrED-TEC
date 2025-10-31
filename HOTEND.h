#ifndef HOTEND_h
#define HOTEND_h

#include <Arduino.h>

const float Raux = 460; 
const float A = 1.1384e-03, B = 2.3245e-04, C = 9.489e-08; //Stainhart-Hart constants

// Parámetros PID ajustados
float integral_H = 0, derivative_H = 0, prevError = 0;
unsigned long prevTime;

float pidMin = 0.0, pidMax = 255.0; //Rango PWM directo
double pid_Hotend;

// Variables de seguridad
float maxTemp = 250.0;
float minTemp = 0.0;


float thermistor(int reading) {
    // Verificar lectura válida
    if (reading <= 10 || reading >= 1020) {
        return -999; // Error: thermistor desconectado
    }
    
    float R = Raux * ((float)reading / (1023.0 - (float)reading));
    float logR = log(R);
    float TempK = 1.0 / (A + B * logR + C * logR * logR * logR);
    return TempK - 273.15;
}

double PIDHotend(float temp, float dt, double setpoint_Hotend, double Kp_H, double Ki_H, double Kd_H){
  // Cálculo PID
  double error_Hotend = setpoint_Hotend - temp;
  integral_H += error_Hotend * dt;
  derivative_H = (error_Hotend - prevError) / dt;
  pid_Hotend = Kp_H * error_Hotend + Ki_H * integral_H + Kd_H * derivative_H;
  prevError = error_Hotend;
  
  // Anti-windup integral CRÍTICO
  if (pid_Hotend > pidMax) {
      pid_Hotend = pidMax;
      integral_H -= error_Hotend * dt; // Prevenir acumulación
  } else if (pid_Hotend < pidMin) {
      pid_Hotend = pidMin;
      integral_H -= error_Hotend * dt; // Prevenir acumulación
  }
  
  // Limitar integral para evitar windup extremo
  integral_H = constrain(integral_H, -50, 50);

  return pid_Hotend;

}

#endif