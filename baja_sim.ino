// baja hil sensor simulator
// simulates hall effect gear tooth sensors (primary + rear)

const uint8_t PRIMARY_PIN = 9;
const uint8_t REAR_PIN = 10;

const uint8_t PRIMARY_TEETH = 36; // primary clutch sheave reluctor
const uint8_t REAR_TEETH = 44;    // rear/wheel output sensor

const uint8_t TEST_SELECTED = 0; // which test to run

// fault types
enum FaultType { NONE = 0, DROPOUT = 1, NOISE = 2, SLIP = 3 };

struct TestCase {
  const char* name;
  float primaryRPM;
  float rearRPM;
  uint8_t primaryFaultType;
  uint8_t faultType;
};

// !!!AI GENERATED TEST CASES lmk if they are good
TestCase tests[] = {
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

void setup() {
  Serial.begin(115200);
  pinMode(PRIMARY_PIN, OUTPUT);
  pinMode(REAR_PIN, OUTPUT);

  randomSeed(analogRead(A0));
  Serial.print("starting test: ");
  Serial.println(tests[TEST_SELECTED].name);
  
}

// half period in us for a given rpm + tooth count
unsigned long halfPeriodUs(float rpm, uint8_t teeth) {
  float freq = (rpm / 60.0) * teeth;
  return (unsigned long)(1000000.0 / (2.0 * freq));
}

void loop() {
  TestCase &t = tests[TEST_SELECTED];

  unsigned long nowUs = micros();

  // primary channel, faults handled here same as rear
  static unsigned long primaryHalfPeriod = halfPeriodUs(t.primaryRPM, PRIMARY_TEETH);
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
    if (nowUs - primaryLastToggle >= primaryHalfPeriod) {
      primaryLastToggle = nowUs;
      primaryState = !primaryState;
      digitalWrite(PRIMARY_PIN, primaryState);
    }
  }

  // rear channel, rpm already encodes slip via test case, faults handled here
  static unsigned long rearHalfPeriod = halfPeriodUs(t.rearRPM, REAR_TEETH);
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
