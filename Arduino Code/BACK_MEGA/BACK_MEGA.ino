/*
 * BACK MEGA - DIRECT PWM CONTROL
 * Receives PWM values directly from ROS
 * 
 * Serial Commands:
 * - "DRIVE,<left_pwm>,<right_pwm>\n"
 * - "STOP\n"
 * 
 * PWM range: -255 to 255
 */
#include <Arduino.h>

// -------- DRIVE MOTOR PINS --------
const int LEFT_R_PWM = 5;   // Left wheel forward
const int LEFT_L_PWM = 4;   // Left wheel reverse
const int RIGHT_R_PWM = 6;  // Right wheel forward
const int RIGHT_L_PWM = 7;  // Right wheel reverse

// -------- CONTROL VARIABLES --------
int targetLeftPWM = 0;
int targetRightPWM = 0;
bool driveEnable = false;

// ============= MOTOR CONTROL =============
void setLeftMotor(int pwm) {
  if (!driveEnable || pwm == 0) {
    analogWrite(LEFT_R_PWM, 0);
    analogWrite(LEFT_L_PWM, 0);
    return;
  }
  
  if (pwm > 0) {
    // Forward
    analogWrite(LEFT_L_PWM, 0);
    analogWrite(LEFT_R_PWM, abs(pwm));
  } else {
    // Reverse
    analogWrite(LEFT_R_PWM, 0);
    analogWrite(LEFT_L_PWM, abs(pwm));
  }
}

void setRightMotor(int pwm) {
  if (!driveEnable || pwm == 0) {
    analogWrite(RIGHT_R_PWM, 0);
    analogWrite(RIGHT_L_PWM, 0);
    return;
  }
  
  if (pwm > 0) {
    // Forward
    analogWrite(RIGHT_L_PWM, 0);
    analogWrite(RIGHT_R_PWM, abs(pwm));
  } else {
    // Reverse
    analogWrite(RIGHT_R_PWM, 0);
    analogWrite(RIGHT_L_PWM, abs(pwm));
  }
}

void stopMotors() {
  analogWrite(LEFT_R_PWM, 0);
  analogWrite(LEFT_L_PWM, 0);
  analogWrite(RIGHT_R_PWM, 0);
  analogWrite(RIGHT_L_PWM, 0);
  driveEnable = false;
}

// ============= SERIAL PARSING =============
void parseSerialCommand() {
  if (!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  
  if (cmd == "STOP") {
    stopMotors();
    Serial.println("STOP");
    return;
  }
  
  if (cmd.startsWith("DRIVE,")) {
    int comma = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma + 1);
    
    if (comma > 0 && comma2 > 0) {
      String leftStr = cmd.substring(comma + 1, comma2);
      String rightStr = cmd.substring(comma2 + 1);
      
      targetLeftPWM = atoi(leftStr.c_str());
      targetRightPWM = atoi(rightStr.c_str());
      
      targetLeftPWM = constrain(targetLeftPWM, -255, 255);
      targetRightPWM = constrain(targetRightPWM, -255, 255);
      
      driveEnable = (targetLeftPWM != 0 || targetRightPWM != 0);
    }
  }
}

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  pinMode(LEFT_R_PWM, OUTPUT);
  pinMode(LEFT_L_PWM, OUTPUT);
  pinMode(RIGHT_R_PWM, OUTPUT);
  pinMode(RIGHT_L_PWM, OUTPUT);
  
  stopMotors();
  
  Serial.println("BACK_MEGA_READY");
}

// ============= LOOP =============
void loop() {
  parseSerialCommand();
  
  if (driveEnable) {
    setLeftMotor(targetLeftPWM);
    setRightMotor(targetRightPWM);
  }
}
