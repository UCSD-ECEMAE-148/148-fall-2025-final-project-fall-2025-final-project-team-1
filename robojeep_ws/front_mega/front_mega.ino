/*********************************************************************
 *  FULL STEERING CONTROL FOR ROBOJEEP — ROS CONTROLLED
 *
 *  Serial protocol (from ROS):
 *      STEER,<angle_radians>,<enable_flag>\n
 *
 *  Example:
 *      STEER,0.523,1      -> steer to +30°
 *      STEER,-0.349,1     -> steer to -20°
 *      STEER,0.0,0        -> disable motor hold
 *
 *  Hardware:
 *      BTS7960:
 *          R_PWM = 7  (forward)
 *          L_PWM = 6  (reverse)
 *
 *      AS5600 ABS encoder (PWM output → pulseIn)
 *          absPin = 2
 *
 *  IMPORTANT:
 *      Includes ROS safety timeout of 300ms.
 *********************************************************************/

#include <Arduino.h>

// --------- Hardware Pins ---------
const int R_PWM = 7;
const int L_PWM = 6;
const int absPin = 2;

// --------- Steering Limits ---------
const float maxRight = 37.0;   // degrees
const float maxLeft  = -24.0;  // degrees
const float allowedError = 1.0;

// --------- Control Variables ---------
float targetAngleDeg = 0.0;  // target angle in DEGREES
bool steeringEnabled = true;

// --------- Filtering ---------
float filteredAngle = 0.0;
float alpha = 0.20;          // low-pass filter weight

// --------- Safety timeout ---------
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 300;   // ms

// ================================================================
// READ ABSOLUTE ANGLE (AS5600 PWM METHOD)
// Returns filtered angle in degrees
// ================================================================
float readAngleDeg() {
  unsigned long highPulse = pulseIn(absPin, HIGH, 3000);  // timeout 3ms

  if (highPulse == 0) {
    // encoder didn't respond
    return filteredAngle;
  }

  // Convert PWM pulse width (1000–2000 us) → 0–360°, then shift center
  float angle = -map(highPulse, 1000, 2000, 0, 360) - 180;

  // Apply low-pass filter
  filteredAngle = filteredAngle + alpha * (angle - filteredAngle);

  return filteredAngle;
}

// ================================================================
// DRIVE MOTOR TOWARD TARGET ANGLE
// ================================================================
void driveSteering(float targetDeg) {
  float currentDeg = readAngleDeg();

  // Clamp target to real physical limits
  if (targetDeg > maxRight) targetDeg = maxRight;
  if (targetDeg < maxLeft)  targetDeg = maxLeft;

  float error = targetDeg - currentDeg;

  // Small deadzone
  if (fabs(error) < allowedError) {
    error = 0.0;
  }

  // If disabled → hold motor off
  if (!steeringEnabled) {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    return;
  }

  // Safety timeout: no recent ROS command → disable
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    return;
  }

  // P-controller
  float Kp = 30.0;
  float motorOut = Kp * error;

  // Clamp to PWM range
  motorOut = constrain(motorOut, -255, 255);

  if (motorOut > 0) {
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, fabs(motorOut));
  } else {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, fabs(motorOut));
  }
}

// ================================================================
// PARSE ROS MESSAGE: "STEER,<angle_rad>,<enable_flag>"
// ================================================================
// ================================================================
// PARSE ROS MESSAGE: "STEER,<angle_rad>,<enable_flag>"
// Compatible with older Arduino String class (no remove(), no toFloat())
// ================================================================
void parseSerialCommand(String msg) {

  msg.trim();

  // Expect "STEER,<angle>,<flag>"
  if (!msg.startsWith("STEER")) {
    Serial.println("ERR: Unknown cmd");
    return;
  }

  // Convert String → C-string for safe parsing
  char buf[50];
  msg.toCharArray(buf, sizeof(buf));

  // Tokenize
  char* token = strtok(buf, ",");   // "STEER"
  token = strtok(NULL, ",");        // angle part

  if (token == NULL) return;

  float angleRad = atof(token);     // convert angle string → float

  token = strtok(NULL, ",");        // flag part
  if (token == NULL) return;

  int enableFlag = atoi(token);     // convert flag string → int

  // Convert radians → degrees
  targetAngleDeg = angleRad * 57.2958;   // rad * 180/pi

  // Enable flag
  steeringEnabled = (enableFlag != 0);

  lastCommandTime = millis();  // reset timeout

  Serial.print("CMD: target = ");
  Serial.print(targetAngleDeg);
  Serial.print(" deg   enabled = ");
  Serial.println(steeringEnabled);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(absPin, INPUT);

  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);

  Serial.println("STEERING CONTROLLER READY");
}

// ================================================================
// LOOP
// ================================================================
void loop() {

  // Process serial commands
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    parseSerialCommand(msg);
  }

  // Control steering
  driveSteering(targetAngleDeg);

  delay(3);
}

