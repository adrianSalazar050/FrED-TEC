#include "MOTORDC.h"
#include "HOTEND.h"
#include "Pin_map.h"
#include <AccelStepper.h>

// GLOBAL VARIABLES
String inputSerial = "";
String digits = "0000";        // [Motor DC, Fan, Extruder, Hotend]

// EXTRUDER MOTOR (STEPPER)
AccelStepper motor2(AccelStepper::DRIVER, 26, 28);
const int enablePin2 = 24;
bool motor2Enabled = false;

// DC MOTOR PID CONTROL
double setpoint_Motor = 20.0;  // Target RPM, max 55
double Kp_M = 25, Ki_M = 2.5, Kd_M = 1.5;

// HOTEND PID CONTROL
double setpoint_Hotend = 190.0;
float Kp_H = 1.8, Ki_H = 0.9, Kd_H = 0.3;

// STATES AND TIMERS
int moto_m = 0, fan_m = 0, heater_m = 0, extruder_m = 0;
unsigned long lastStatusUpdate = 0;

int fanPWM = 0;                // Fan PWM value (0-255)

// INITIAL SETUP
void setup() {
  Serial.begin(115200);
  Serial.println("Connection established with custom PID control.");

  pinMode(pinFan, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(pinHotend, OUTPUT);
  pinMode(pinMotor, OUTPUT);
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  digitalWrite(enablePin2, HIGH);
  digitalWrite(pinHotend, LOW);
  analogWrite(pinMotor, 0);

  motor2.setMaxSpeed(2000);
  motor2.setAcceleration(1000);

  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);

  prevTime = millis();
}

// MAIN LOOP
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processInput(inputSerial);
      inputSerial = "";
    } else {
      inputSerial += c;
    }
  }

  unsigned long now = millis();
  if (now - prevTime >= 100) {
    float dt = (now - prevTime) / 1000.0;
    prevTime = now;

    float temp_current = thermistor(analogRead(termPin));
    computeRpm();

    // HOTEND PID
    if (digits.length() >= 4 && digits[3] == '1') {
      if (temp_current == -999 || temp_current > maxTemp || temp_current < 10) {
        digitalWrite(pinHotend, LOW);
        integral_H = 0;
        Serial.println("SAFETY SHUTDOWN!");
      } else {
        double pidPWM_Hotend = PIDHotend(temp_current, dt, setpoint_Hotend, Kp_H, Ki_H, Kd_H);
        int pwmTemp = constrain((int)pidPWM_Hotend, 0, 255);
        analogWrite(pinHotend, pwmTemp);
      }
      heater_m = 1;
    } else {
      analogWrite(pinHotend, 0);
      heater_m = 0;
    }

    // DC MOTOR PID
    if (digits.length() >= 1 && digits[0] == '1') {
      input = N;
      double pidPWM_Motor = PIDMotor(dt, setpoint_Motor, Kp_M, Ki_M, Kd_M);
      int pwmMotorOut = constrain((int)pidPWM_Motor, 10, 255);
      analogWrite(pinMotor, pwmMotorOut);
      moto_m = 1;
    } else {
      analogWrite(pinMotor, 0);
      moto_m = 0;
    }
  }

  // FAN
  if (digits.length() >= 2 && digits[1] == '1') {
    analogWrite(pinFan, fanPWM);
    fan_m = (fanPWM > 0) ? 1 : 0;
  } else {
    analogWrite(pinFan, 0);
    fan_m = 0;
  }

  // EXTRUDER
  if (digits.length() >= 3 && digits[2] == '1') {
    if (!motor2Enabled) {
      digitalWrite(enablePin2, LOW);
      motor2Enabled = true;
    }
    motor2.runSpeed();
    extruder_m = 1;
  } else {
    if (motor2Enabled) {
      digitalWrite(enablePin2, HIGH);
      motor2Enabled = false;
    }
    extruder_m = 0;
  }

  // STATUS REPORT
  unsigned long currentMillis = millis();
  if (currentMillis - lastStatusUpdate >= 1000) {
    lastStatusUpdate = currentMillis;
    Serial.print("Temp:"); Serial.println(thermistor(analogRead(termPin)));
    Serial.print("Motor DC RPM:"); Serial.println(N);
    Serial.println("--- Component status ---");
    Serial.print("Fan:        "); Serial.println(fan_m ? "On" : "Off");
    Serial.print("Heater:     "); Serial.println(heater_m ? "On" : "Off");
    Serial.print("Extruder:   "); Serial.println(extruder_m ? "On" : "Off");
    Serial.println("-------------------------");
  }
}

// SERIAL COMMAND PARSING
void processInput(String command) {
  Serial.print("Received: ");
  Serial.println(command);

  if (command.startsWith("ACTUATE:")) {
    digits = command.substring(8);
  }
  else if (command.startsWith("SPEED:")) {
    int newSpeed = command.substring(6).toInt();
    motor2.setSpeed(newSpeed);
    Serial.print("New extruder speed: "); Serial.println(newSpeed);
  }
  else if (command.startsWith("TEMP:")) {
    double newTemp = command.substring(5).toDouble();
    setpoint_Hotend = newTemp;
    Serial.print("New temperature setpoint: "); Serial.println(setpoint_Hotend);
  }
  else if (command.startsWith("DCSPEED:")) {
    double newDCSpeed = command.substring(8).toDouble();
    setpoint_Motor = newDCSpeed;
    Serial.print("New DC motor RPM setpoint: "); Serial.println(setpoint_Motor);
  }
  else if (command.startsWith("FANSPEED:")) {
    int newFanSpeed = command.substring(9).toInt();
    fanPWM = map(newFanSpeed, 0, 100, 0, 255);
    Serial.print("New fan PWM: "); Serial.println(fanPWM);
  }

  // Hotend PID gains command
  else if (command.startsWith("PIDH:")) {
    String values = command.substring(5);
    int firstComma = values.indexOf(',');
    int secondComma = values.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      Kp_H = values.substring(0, firstComma).toFloat();
      Ki_H = values.substring(firstComma + 1, secondComma).toFloat();
      Kd_H = values.substring(secondComma + 1).toFloat();

      Serial.println("Hotend PID gains updated:");
      Serial.print("Kp: "); Serial.print(Kp_H, 4);
      Serial.print(", Ki: "); Serial.print(Ki_H, 4);
      Serial.print(", Kd: "); Serial.println(Kd_H, 4);
    }
  }
  // DC motor PID gains command
  else if (command.startsWith("PIDM:")) {
    String values = command.substring(5);
    int firstComma = values.indexOf(',');
    int secondComma = values.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      Kp_M = values.substring(0, firstComma).toFloat();
      Ki_M = values.substring(firstComma + 1, secondComma).toFloat();
      Kd_M = values.substring(secondComma + 1).toFloat();

      Serial.println("DC motor PID gains updated:");
      Serial.print("Kp: "); Serial.print(Kp_M, 4);
      Serial.print(", Ki: "); Serial.print(Ki_M, 4);
      Serial.print(", Kd: "); Serial.println(Kd_M, 4);
    }
  }
}
