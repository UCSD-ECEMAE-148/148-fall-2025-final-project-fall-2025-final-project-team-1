/*
 * FRONT MEGA - DIRECT PWM CONTROL + STEERING
 * Receives PWM values directly from ROS
 * 
 * Serial Commands:
 * - "DRIVE,<left_pwm>,<right_pwm>\n"
 * - "STEER,<angle_rad>,<enable>\n"
 * - "STOP\n"
 * 
 * PWM range: -255 to 255
 */
#include <Arduino.h>

// -------- DRIVE MOTOR PINS --------
const int LEFT_R_PWM = 4;   // Left wheel forward
const int LEFT_L_PWM = 5;   // Left wheel reverse
const int RIGHT_R_PWM = 6;  // Right wheel forward
const int RIGHT_L_PWM = 7;  // Right wheel reverse

// -------- STEERING MOTOR PINS --------
const int STEER_R_PWM = 10;  // Steering forward/right
const int STEER_L_PWM = 9;   // Steering reverse/left
const int STEER_ENCODER_PIN = 2;  // Steering angle encoder (PWM input)

// -------- CONTROL VARIABLES --------
int targetLeftPWM = 0;
int targetRightPWM = 0;
bool driveEnable = false;

// -------- STEERING CONTROL VARIABLES --------
float steeringTargetAngle = 0.0;  // Target angle in degrees
bool steeringEnable = false;      // Is steering enabled?
float filteredAngle = 0.0;        // Current smoothed angle
const float alpha = 0.2;          // Smoothing filter constant
const float maxRight = 60.0;      // Max right turn angle (degrees)
const float maxLeft = -60.0;      // Max left turn angle (degrees)
const float allowedError = 1.0;   // Dead zone (degrees)
const float Kp = 30.0;            // P-controller gain
float angleOffset = -10.0;          // Calibration offset
float MAX_STEERING_POWER = 1.0;   // Power limit (0.0 to 1.0) - START LOW!

// ============= STEERING ENCODER READING =============
float readSteeringAngle() {
  unsigned long highTime = pulseIn(STEER_ENCODER_PIN, HIGH, 1500);
  
  if (highTime == 0) {
    // No valid pulse - return last good value
    return filteredAngle;
  }
  
  // Convert pulse width to angle
  float angle = -map(highTime, 1000, 2000, 0, 360) - 180 + angleOffset;
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {  // Print every 500ms
    Serial.print("Pulse: ");
    Serial.print(highTime);
    Serial.print(" us | Angle: ");
    Serial.print(filteredAngle);
    Serial.println(" deg");
    lastPrint = millis();
  }
  
  // Apply smoothing filter
  filteredAngle = filteredAngle + alpha * (angle - filteredAngle);
  
  return filteredAngle;
}

// ============= STEERING MOTOR CONTROL =============
void driveToAngle(float targetDeg) {
  float current = readSteeringAngle();
  
  if (targetDeg > maxRight) {
    targetDeg = maxRight;
  }
  else if (targetDeg < maxLeft) {
    targetDeg = maxLeft;
  }
  
  float error = targetDeg - current;
  
  if ((error < allowedError) && (error > -allowedError)) {
    error = 0.0;
  }
  
  if (!steeringEnable) {
    error = 0.0;
  }
  
  float motor = Kp * error;
  motor = constrain(motor, -255, 255);
  motor = motor * MAX_STEERING_POWER;
  
  if (motor > 0) {
    analogWrite(STEER_L_PWM, 0);
    analogWrite(STEER_R_PWM, fabs(motor));
  } else {
    analogWrite(STEER_R_PWM, 0);
    analogWrite(STEER_L_PWM, fabs(motor));
  }
}

void stopSteeringMotor() {
  analogWrite(STEER_R_PWM, 0);
  analogWrite(STEER_L_PWM, 0);
}

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
    stopSteeringMotor();
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
    return;
  }
  
  if (cmd.startsWith("STEER,")) {
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma1 + 1);
    
    if (comma1 > 0 && comma2 > 0) {
      String angleStr = cmd.substring(comma1 + 1, comma2);
      String enableStr = cmd.substring(comma2 + 1);
      
      float angleRad = atof(angleStr.c_str());
      int enableFlag = atoi(enableStr.c_str());
      
      steeringTargetAngle = angleRad * 57.2958;
      steeringEnable = (enableFlag == 1);
    }
    return;
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
  
  pinMode(STEER_R_PWM, OUTPUT);
  pinMode(STEER_L_PWM, OUTPUT);
  pinMode(STEER_ENCODER_PIN, INPUT);
  
  stopMotors();
  stopSteeringMotor();
  
  Serial.println("READY");
  
  // Initialize steering angle filter with several reads
  for (int i = 0; i < 20; i++) {
    readSteeringAngle();
    delay(5);
  }
}

// ============= LOOP =============
void loop() {
  parseSerialCommand();
  
  if (driveEnable) {
    setLeftMotor(targetLeftPWM);
    setRightMotor(targetRightPWM);
  }
  
  driveToAngle(steeringTargetAngle);
}
