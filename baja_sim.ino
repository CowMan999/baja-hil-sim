// baja hil sensor simulator
// simulates hall effect gear tooth sensors (primary + rear)
// now with 2 pots (A0/A1) that shift primary/rear rpm relative to the
// selected test case's baseline values

const uint8_t PRIMARY_PIN = 9;
const uint8_t REAR_PIN = 10;

const uint8_t PRIMARY_TEETH = 36; // primary clutch sheave reluctor
const uint8_t REAR_TEETH = 44;    // rear/wheel output sensor

const uint8_t TEST_SELECTED = 0; // which test to run

// pots: wired 5v -> wiper -> A0/A1 -> gnd (standard 3-pin pot divider)
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
  {"low",           50,  20, NONE,    NONE   }, // low for led test (dont use this)
  {"idle",           1800,  600, NONE,    NONE   }, // engine idle, clutch barely engaged
  {"engagement",     2200, 1200, NONE,    NONE   }, // clutch starting to bite
  {"cruise",         3200, 2800, NONE,    NONE   }, // mid cvt ratio, normal driving
  {"wot_redline",     3800, 3600, NONE,    NONE   }, // wot, near 1:1 top ratio
  {"belt_slip",       3800, 1500, NONE,    SLIP   }, // belt slipping, primary spins rear lags
  {"rear_dropout",    3200, 2800, NONE,    DROPOUT}, // broken wire / missed teeth on rear
  {"sensor_noise",    3200, 2800, NONE,    NOISE  }, // emi glitches on rear signal
  {"primary_dropout", 3200, 2800, DROPOUT, NONE   }, // broken wire / missed teeth on primary
  {"primary_noise",   3200, 2800, NOISE,   NONE   }, // emi glitches on primary signal
};

const uint8_t NUM_TESTS = sizeof(tests) / sizeof(tests[0]);

unsigned long primaryLastToggle = 0;
unsigned long rearLastToggle = 0;
bool primaryState = LOW;
bool rearState = LOW;

// these used to be `static` locals inside loop(); pulled out to file scope
// so the pot-sampling block can update them too
unsigned long primaryHalfPeriod = 0;
unsigned long rearHalfPeriod = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PRIMARY_PIN, OUTPUT);
  pinMode(REAR_PIN, OUTPUT);

  randomSeed(analogRead(A2)); // A0/A1 now taken by the pots, seed off a floating pin instead

  Serial.print("starting test: ");
  Serial.println(tests[TEST_SELECTED].name);

  // seed initial periods off the raw baseline before the first pot sample runs
  primaryHalfPeriod = halfPeriodUs(tests[TEST_SELECTED].primaryRPM, PRIMARY_TEETH);
  rearHalfPeriod = halfPeriodUs(tests[TEST_SELECTED].rearRPM, REAR_TEETH);
}

// half period in us for a given rpm + tooth count
unsigned long halfPeriodUs(float rpm, uint8_t teeth) {
  if (rpm <= 0) return 0; // avoid div by zero / negative freq, treat as "stopped"
  float freq = (rpm / 60.0) * teeth;
  return (unsigned long)(1000000.0 / (2.0 * freq));
}

// map a raw analogRead (0-1023) to a +/- RPM_SHIFT_RANGE offset
float mapPotToShift(int raw) {
  if(raw < 10) return 0;
  else return ((float)raw / 1023.0) * (2.0 * RPM_SHIFT_RANGE) - RPM_SHIFT_RANGE;
}

void loop() {
  TestCase &t = tests[TEST_SELECTED];

  unsigned long nowUs = micros();

  // resample pots + recompute toggle periods every loop
  int primaryPotRaw = analogRead(PRIMARY_POT_PIN);
  int rearPotRaw = analogRead(REAR_POT_PIN);

  primaryRPMShift = mapPotToShift(primaryPotRaw);
  rearRPMShift = mapPotToShift(rearPotRaw);

  float effectivePrimaryRPM = t.primaryRPM + primaryRPMShift;
  float effectiveRearRPM = t.rearRPM + rearRPMShift;
  if (effectivePrimaryRPM < 0) effectivePrimaryRPM = 0;
  if (effectiveRearRPM < 0) effectiveRearRPM = 0;

  primaryHalfPeriod = halfPeriodUs(effectivePrimaryRPM, PRIMARY_TEETH);
  rearHalfPeriod = halfPeriodUs(effectiveRearRPM, REAR_TEETH);

  // primary channel, faults handled here same as rear
  if (t.primaryFaultType == DROPOUT) {
    if (primaryHalfPeriod > 0 && nowUs - primaryLastToggle >= primaryHalfPeriod) {
      primaryLastToggle = nowUs;
      if (random(0, 100) > 15) { // ~15% of edges dropped
        primaryState = !primaryState;
        digitalWrite(PRIMARY_PIN, primaryState);
      }
    }
  } else if (t.primaryFaultType == NOISE) {
    if (primaryHalfPeriod > 0 && nowUs - primaryLastToggle >= primaryHalfPeriod) {
      primaryLastToggle = nowUs;
      primaryState = !primaryState;
      digitalWrite(PRIMARY_PIN, primaryState);
    }
    if (random(0, 1000) < 3) { // rare random glitch edge
      digitalWrite(PRIMARY_PIN, !digitalRead(PRIMARY_PIN));
    }
  } else {
    if (primaryHalfPeriod > 0 && nowUs - primaryLastToggle >= primaryHalfPeriod) {
      primaryLastToggle = nowUs;
      primaryState = !primaryState;
      digitalWrite(PRIMARY_PIN, primaryState);
    }
  }

  // rear channel, rpm already encodes slip via test case, faults handled here
  if (t.faultType == DROPOUT) {
    if (rearHalfPeriod > 0 && nowUs - rearLastToggle >= rearHalfPeriod) {
      rearLastToggle = nowUs;
      if (random(0, 100) > 15) { // ~15% of edges dropped
        rearState = !rearState;
        digitalWrite(REAR_PIN, rearState);
      }
    }
  } else if (t.faultType == NOISE) {
    if (rearHalfPeriod > 0 && nowUs - rearLastToggle >= rearHalfPeriod) {
      rearLastToggle = nowUs;
      rearState = !rearState;
      digitalWrite(REAR_PIN, rearState);
    }
    if (random(0, 1000) < 3) { // rare random glitch edge
      digitalWrite(REAR_PIN, !digitalRead(REAR_PIN));
    }
  } else {
    // covers NONE and SLIP, slip is just the rpm mismatch in the test case
    if (rearHalfPeriod > 0 && nowUs - rearLastToggle >= rearHalfPeriod) {
      rearLastToggle = nowUs;
      rearState = !rearState;
      digitalWrite(REAR_PIN, rearState);
    }
  }
}
