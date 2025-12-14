#include <Wire.h>
#include <AS5600.h>

AS5600 encoder;

// -------- Angle smoothing --------
float smoothAngle = 0.0f;
const float alphaAngle = 0.2f;

// -------- RPM smoothing --------
float smoothRPM = 0.0f;
const float alphaRPM = 0.2f;

// Per-sample state
uint16_t lastRaw = 0;
unsigned long lastTime = 0;

// ---------- Sliding window for RPM ----------
const uint8_t WINDOW_SIZE = 32;
int16_t  deltaBuf[WINDOW_SIZE];
uint16_t dtBuf[WINDOW_SIZE];

int32_t  sumCounts = 0;
uint32_t sumDtUs   = 0;

uint8_t  winIndex  = 0;
bool     winFilled = false;

// ---------- Measurement timing ----------
const unsigned long MEASURE_DURATION_MS = 10000; // 10 seconds
unsigned long measureStart = 0;
bool measuring = true;

// ---------- Statistics ----------

// Raw min/max (no filtering at all)
float rawMin =  1e9;
float rawMax = -1e9;

// Robust stats (filtered via ±3σ)
uint32_t sampleCount = 0;
double   meanRPM = 0.0;
double   M2 = 0.0;            // accumulator for variance

// sigma-filtered min/max
float robustMin =  1e9;
float robustMax = -1e9;

const float SIGMA_THRESHOLD = 3.0f;


// ---------- Update Robust Statistics ----------
void updateStatsRobust(float x) {
  // Always update raw min/max so we can see total spread
  if (x < rawMin) rawMin = x;
  if (x > rawMax) rawMax = x;

  // Welford online mean/variance
  sampleCount++;
  double delta  = x - meanRPM;
  meanRPM      += delta / sampleCount;
  double delta2 = x - meanRPM;
  M2           += delta * delta2;

  // Need a few samples before variance is meaningful
  if (sampleCount < 5) return;

  double variance = M2 / (sampleCount - 1);
  double sigma = sqrt(variance);

  // Reject strong outliers beyond ±3 sigma
  if (fabs(x - meanRPM) > SIGMA_THRESHOLD * sigma)
    return;

  // Update robust min/max (for inlier data only)
  if (x < robustMin) robustMin = x;
  if (x > robustMax) robustMax = x;
}


void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);

  lastRaw  = encoder.rawAngle();
  smoothAngle = lastRaw * 360.0f / 4096.0f;
  lastTime = micros();

  // Initialize sliding window
  for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
    deltaBuf[i] = 0;
    dtBuf[i]    = 1;
  }

  measureStart = millis();
}


void loop() {
  if (measuring) {

    // -------- Read sensor --------
    uint16_t raw = encoder.rawAngle();
    unsigned long now = micros();
    unsigned long dtUs = now - lastTime;
    if (dtUs == 0) return;   // safety

    // -------- Angle smoothing --------
    float angle = raw * 360.0f / 4096.0f;

    float diffAngle = angle - smoothAngle;
    if (diffAngle > 180.0f)  diffAngle -= 360.0f;
    if (diffAngle < -180.0f) diffAngle += 360.0f;
    smoothAngle += alphaAngle * diffAngle;

    if (smoothAngle < 0.0f)  smoothAngle += 360.0f;
    if (smoothAngle >= 360.0f) smoothAngle -= 360.0f;

    // -------- Delta counts with wrap --------
    int16_t deltaCounts = (int16_t)raw - (int16_t)lastRaw;
    if (deltaCounts > 2048)       deltaCounts -= 4096;
    else if (deltaCounts < -2048) deltaCounts += 4096;

    // -------- Sliding window update --------
    sumCounts -= deltaBuf[winIndex];
    sumDtUs   -= dtBuf[winIndex];

    deltaBuf[winIndex] = deltaCounts;
    dtBuf[winIndex]    = dtUs;

    sumCounts += deltaCounts;
    sumDtUs   += dtUs;

    winIndex++;
    if (winIndex >= WINDOW_SIZE) {
      winIndex = 0;
      winFilled = true;
    }

    // -------- Compute RPM (from window) --------
    float rpm = 0.0f;
    if (winFilled && sumDtUs > 0) {
      float deltaRevs = sumCounts / 4096.0f;
      float windowDt  = sumDtUs / 1e6f;
      float rps = deltaRevs / windowDt;
      rpm = rps * 60.0f;
    }

    // Smooth RPM
    smoothRPM += alphaRPM * (rpm - smoothRPM);

    // -------- Update stats using smoothed RPM --------
    if (winFilled) updateStatsRobust(smoothRPM);

    // -------- End after 10 seconds --------
    if (millis() - measureStart >= MEASURE_DURATION_MS) {
      measuring = false;
    }

    lastRaw  = raw;
    lastTime = now;

  } else {
    // -------- PRINT RESULTS ONE TIME --------
    double variance = (sampleCount > 1) ? (M2 / (sampleCount - 1)) : 0.0;
    double stddev = sqrt(variance);

    // Approx 95% bounds assuming near-normal distribution
    double lower95 = meanRPM - 1.96 * stddev;
    double upper95 = meanRPM + 1.96 * stddev;

    Serial.println("\n==== RPM STATISTICS OVER 10 SECONDS ====");

    Serial.print("Samples analyzed: ");
    Serial.println(sampleCount);

    Serial.print("\nRAW Min RPM (includes all glitches): ");
    Serial.println(rawMin, 2);

    Serial.print("RAW Max RPM (includes all glitches): ");
    Serial.println(rawMax, 2);

    Serial.print("\nRobust Min RPM (outliers removed): ");
    Serial.println(robustMin, 2);

    Serial.print("Robust Max RPM (outliers removed): ");
    Serial.println(robustMax, 2);

    Serial.print("\nMean RPM: ");
    Serial.println(meanRPM, 2);

    Serial.print("Standard Deviation RPM: ");
    Serial.println(stddev, 2);

    Serial.print("\nApprox 95% RPM lower bound (mean - 1.96*σ): ");
    Serial.println(lower95, 2);

    Serial.print("Approx 95% RPM upper bound (mean + 1.96*σ): ");
    Serial.println(upper95, 2);

    while (1); // halt
  }
}
