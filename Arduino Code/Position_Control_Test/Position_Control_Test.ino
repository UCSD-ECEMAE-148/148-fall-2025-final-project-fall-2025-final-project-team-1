#include <Wire.h>
#include <AS5600.h>

AS5600 encoder;

// ---- Encoder / gearing config ----
const int32_t COUNTS_PER_REV   = 4096;   // AS5600 counts per motor shaft revolution
const float   GEAR_RATIO       = 101.8;  // Motor revs per 1 output gear rev (your measured ratio)
const float   TARGET_WHEEL_REV = 5.0;    // Desired output gear rotations

// ---- BTS7960 pins ----
const int R_PWM = 5;   // Forward PWM
const int L_PWM = 6;   // Reverse PWM
// R_EN and L_EN are wired directly to 5V in hardware

// ---- Motor helpers ----
void motor_forward(int pwm) {
  analogWrite(R_PWM, pwm);
  analogWrite(L_PWM, 0);
}

// Strong electrical braking (shorts motor via H-bridge)
void motor_brake() {
  analogWrite(R_PWM, 255);
  analogWrite(L_PWM, 255);
}

// Let motor coast (no drive)
void motor_coast() {
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);  // fast I2C for AS5600

  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);

  delay(500);  // just to let everything power up and settle (motor is OFF here)

  // ---- Compute target total motor counts ----
  const float targetMotorRevsF = TARGET_WHEEL_REV * GEAR_RATIO;
  const long  targetMotorRevs  = (long)(targetMotorRevsF + 0.5f);  // round to nearest
  const long  targetCounts     = targetMotorRevs * COUNTS_PER_REV;

  Serial.println("Starting 5 wheel rotations test (no extra pre-motion)...");
  Serial.print("Target motor revolutions (rounded): ");
  Serial.println(targetMotorRevs);
  Serial.print("Target encoder counts: ");
  Serial.println(targetCounts);

  // ---- Initialize encoder integration state right before motion ----
  long     cumCounts = 0;
  uint16_t lastRaw   = encoder.rawAngle();

  // ---- Turn motor on and IMMEDIATELY start counting ----
  const int DRIVE_PWM = 200;  // tune speed if you want slower/faster
  motor_forward(DRIVE_PWM);

  while (labs(cumCounts) < targetCounts) {
    uint16_t raw = encoder.rawAngle();

    // Signed delta with wrap-around handling (0..4095)
    int16_t delta = (int16_t)raw - (int16_t)lastRaw;
    if (delta > 2048)       delta -= 4096;
    else if (delta < -2048) delta += 4096;

    cumCounts += delta;
    lastRaw = raw;

    // (Optional) uncomment for debugging
    // Serial.println(cumCounts);
  }

  // ---- Stop motor with braking, then release ----
  motor_brake();   // lock it quickly
  delay(100);      // let it come fully to rest
  motor_coast();   // then relax the bridge

  Serial.println("\nDone! Reached target counts.");

  float motorRevs = (float)cumCounts / (float)COUNTS_PER_REV;
  float wheelRevs = motorRevs / GEAR_RATIO;

  Serial.print("Total motor revs measured: ");
  Serial.println(motorRevs, 4);

  Serial.print("Total wheel revs measured (based on ratio): ");
  Serial.println(wheelRevs, 4);

  Serial.println("\nTest complete.");
}

void loop() {
  // nothing here
}
