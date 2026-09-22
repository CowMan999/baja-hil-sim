// baja hil sensor simulator
// simulates hall effect gear tooth sensors (primary + rear)

const uint8_t PRIMARY_PIN = 9;
const uint8_t REAR_PIN = 10;

const float PRIMARY_PPR = 4.0;
const float REAR_PPR    = 6.0;

const uint8_t TEST_SELECTED = 1; // which test to run

const bool USE_POTS = false; // leave false if pots aren't actually wired up
const uint8_t PRIMARY_POT_PIN = A0;
const uint8_t REAR_POT_PIN = A1;

// how far each pot can push rpm away from the test case's baseline value,
// in either direction. pot centered (~512) = no shift, full ccw = -RANGE,
// full cw = +RANGE
const float RPM_SHIFT_RANGE = 800.0;

// current shift values, kept around for debug printing
float primaryRPMShift = 0;
float rearRPMShift = 0;

// fault types
enum FaultType { NONE = 0, DROPOUT = 1, NOISE = 2, SLIP = 3 };

struct TestCase {
  const char* name;
  float primaryRPM;
  float rearRPM;
  uint8_t primaryFaultType;
  uint8_t faultType;
};

TestCase tests[] = {
  {"low",             200,  100, NONE,    NONE   }, // low for led test (dont use this)
  {"idle",           1800,  600, NONE,    NONE   }, // engine idle, clutch barely engaged
  {"engagement",     2200, 1200, NONE,    NONE   }, // clutch starting to bite
  {"cruise",         3200, 2800, NONE,    NONE   }, // mid cvt ratio, normal driving
  {"wot_redline",    3800, 3600, NONE,    NONE   }, // wot, near 1:1 top ratio
  {"belt_slip",      3800, 1500, NONE,    SLIP   }, // belt slipping, primary spins rear lags
  {"rear_dropout",   3200, 2800, NONE,    DROPOUT}, // broken wire / missed teeth on rear
  {"sensor_noise",   3200, 2800, NONE,    NOISE  }, // emi glitches on rear signal
  {"primary_dropout",3200, 2800, DROPOUT, NONE   }, // broken wire / missed teeth on primary
  {"primary_noise",  3200, 2800, NOISE,   NONE   }, // emi glitches on primary signal
};

const uint8_t NUM_TESTS = sizeof(tests) / sizeof(tests[0]);

unsigned long primaryLastToggle = 0;
unsigned long rearLastToggle = 0;

bool primaryLineLow = false; // false = released/high, true = actively sunk low
bool rearLineLow = false;

unsigned long primaryHalfPeriod = 0;
unsigned long rearHalfPeriod = 0;

// --- open-drain helpers -----------------------------------------------
inline void driveLow(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
inline void driveRelease(uint8_t pin) {
  pinMode(pin, INPUT); // high-Z
}

// half period in us for a given rpm + PPR (pulses per rev the LOGGER
// expects, not physical teeth count)
unsigned long halfPeriodUs(float rpm, float ppr) {
  if (rpm <= 0) return 0; // treat as "stopped"
  float freq = (rpm / 60.0) * ppr;
  if (freq <= 0) return 0;
  return (unsigned long)(1000000.0 / (2.0 * freq));
}

// map a raw analogRead (0-1023) to a +/- RPM_SHIFT_RANGE offset.
// continuous across the full range, no cliff at the bottom.
float mapPotToShift(int raw) {
  if(raw < 10) return 0;
  return ((float)raw / 1023.0) * (2.0 * RPM_SHIFT_RANGE) - RPM_SHIFT_RANGE;
}

void setup() {
  Serial.begin(115200);

  // start both lines released (idle high via the board's pull-up)
  driveRelease(PRIMARY_PIN);
  driveRelease(REAR_PIN);

  randomSeed(analogRead(A2)); // A0/A1 taken by pots, seed off a floating pin

  Serial.print("starting test: ");
  Serial.println(tests[TEST_SELECTED].name);

  primaryHalfPeriod = halfPeriodUs(tests[TEST_SELECTED].primaryRPM, PRIMARY_PPR);
  rearHalfPeriod = halfPeriodUs(tests[TEST_SELECTED].rearRPM, REAR_PPR);
}

void loop() {
  TestCase &t = tests[TEST_SELECTED];

  unsigned long nowUs = micros();

  if (USE_POTS) {
    primaryRPMShift = mapPotToShift(analogRead(PRIMARY_POT_PIN));
    rearRPMShift = mapPotToShift(analogRead(REAR_POT_PIN));
  } else {
    primaryRPMShift = 0;
    rearRPMShift = 0;
  }

  float effectivePrimaryRPM = t.primaryRPM + primaryRPMShift;
  float effectiveRearRPM = t.rearRPM + rearRPMShift;
  if (effectivePrimaryRPM < 0) effectivePrimaryRPM = 0;
  if (effectiveRearRPM < 0) effectiveRearRPM = 0;

  primaryHalfPeriod = halfPeriodUs(effectivePrimaryRPM, PRIMARY_PPR);
  rearHalfPeriod = halfPeriodUs(effectiveRearRPM, REAR_PPR);

  // ---------------- primary channel ----------------
  if (t.primaryFaultType == DROPOUT) {
    if (primaryHalfPeriod > 0 && nowUs - primaryLastToggle >= primaryHalfPeriod) {
      primaryLastToggle = nowUs;
      if (random(0, 100) > 15) { // ~15% of edges dropped
        primaryLineLow = !primaryLineLow;
        primaryLineLow ? driveLow(PRIMARY_PIN) : driveRelease(PRIMARY_PIN);
      }
    }
  } else if (t.primaryFaultType == NOISE) {
    if (primaryHalfPeriod > 0 && nowUs - primaryLastToggle >= primaryHalfPeriod) {
      primaryLastToggle = nowUs;
      primaryLineLow = !primaryLineLow;
      primaryLineLow ? driveLow(PRIMARY_PIN) : driveRelease(PRIMARY_PIN);
    }
    if (random(0, 1000) < 3) { // rare random glitch edge
      primaryLineLow = !primaryLineLow;
      primaryLineLow ? driveLow(PRIMARY_PIN) : driveRelease(PRIMARY_PIN);
    }
  } else {
    if (primaryHalfPeriod > 0 && nowUs - primaryLastToggle >= primaryHalfPeriod) {
      primaryLastToggle = nowUs;
      primaryLineLow = !primaryLineLow;
      primaryLineLow ? driveLow(PRIMARY_PIN) : driveRelease(PRIMARY_PIN);
    }
  }

  // ---------------- rear channel ----------------
  // (NONE and SLIP both fall through to the plain toggle branch below;
  // SLIP is just the rpm mismatch baked into the test case)
  if (t.faultType == DROPOUT) {
    if (rearHalfPeriod > 0 && nowUs - rearLastToggle >= rearHalfPeriod) {
      rearLastToggle = nowUs;
      if (random(0, 100) > 15) {
        rearLineLow = !rearLineLow;
        rearLineLow ? driveLow(REAR_PIN) : driveRelease(REAR_PIN);
      }
    }
  } else if (t.faultType == NOISE) {
    if (rearHalfPeriod > 0 && nowUs - rearLastToggle >= rearHalfPeriod) {
      rearLastToggle = nowUs;
      rearLineLow = !rearLineLow;
      rearLineLow ? driveLow(REAR_PIN) : driveRelease(REAR_PIN);
    }
    if (random(0, 1000) < 3) {
      rearLineLow = !rearLineLow;
      rearLineLow ? driveLow(REAR_PIN) : driveRelease(REAR_PIN);
    }
  } else {
    if (rearHalfPeriod > 0 && nowUs - rearLastToggle >= rearHalfPeriod) {
      rearLastToggle = nowUs;
      rearLineLow = !rearLineLow;
      rearLineLow ? driveLow(REAR_PIN) : driveRelease(REAR_PIN);
    }
  }
}
