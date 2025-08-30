/*
  One-Motor: Calibrate + Learn + Run
  - ESC calibration (MAX then MIN)
  - RC throttle range learn (stores to EEPROM)
  - Normal run uses stored RC range and maps to ESC range

  Wiring:
    RC Throttle (CH3 PWM) -> D2 (input)
    ESC signal            -> D9 (output)
    Receiver/BEC 5V -> 5V  (or power Arduino by USB; share GND)
    GND shared across Receiver, ESC, Arduino

  Serial commands:
    'c' = ESC calibration
    'l' = Learn RC throttle range (EEPROM)
    'n' = Normal run (uses learned RC range)
    's' = 2s spin test at TEST_US
*/

#include <Servo.h>
#include <EEPROM.h>

/********* USER PINS *********/
const uint8_t PIN_RC_THROTTLE = 2;   // RC CH3 input
const uint8_t PIN_ESC_SIGNAL  = 9;   // ESC output

/********* ESC ENDPOINTS (typical) *********/
const int ESC_MIN_US   = 1000;
const int ESC_MAX_US   = 2000;
const int ESC_IDLE_US  = 1100;   // set to ESC_MIN_US if you want motor fully stopped at idle
const int TEST_US      = 1350;   // spin test level

/********* RC READ SETTINGS *********/
const unsigned long PULSE_TIMEOUT_US = 25000;  // 25ms; RC frame ~20ms
const unsigned long FAILSAFE_MS      = 300;

/********* SMOOTHING *********/
const float SMOOTH_ALPHA = 0.25f;    // 0..1 (higher = less smoothing)
const unsigned long PRINT_MS = 150;

/********* EEPROM LAYOUT *********/
const int   EE_MAGIC_ADDR = 0;    // 1 byte
const int   EE_RC_LOW     = 2;    // int (2 bytes)
const int   EE_RC_HIGH    = 4;    // int (2 bytes)
const byte  EE_MAGIC_VAL  = 0x42; // "valid" marker

/********* GLOBALS *********/
Servo esc;
int rcLowUs  = 1100;   // defaults until learned
int rcHighUs = 1900;
unsigned long lastSignalMs = 0;
unsigned long lastPrintMs  = 0;
int filteredUs = ESC_MIN_US;

/********* UTILS *********/
int clamp(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

int readPulseUs(uint8_t pin) {
  pinMode(pin, INPUT);
  unsigned long w = pulseIn(pin, HIGH, PULSE_TIMEOUT_US);
  return (w == 0) ? 0 : (int)w;
}

long mapLong(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void eepromWriteInt(int addr, int val) { EEPROM.put(addr, val); }
int  eepromReadInt(int addr) { int v; EEPROM.get(addr, v); return v; }

void loadRcRangeFromEEPROM() {
  byte m = EEPROM.read(EE_MAGIC_ADDR);
  if (m == EE_MAGIC_VAL) {
    rcLowUs  = eepromReadInt(EE_RC_LOW);
    rcHighUs = eepromReadInt(EE_RC_HIGH);
  }
}

void saveRcRangeToEEPROM(int lowUs, int highUs) {
  EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC_VAL);
  eepromWriteInt(EE_RC_LOW,  lowUs);
  eepromWriteInt(EE_RC_HIGH, highUs);
}

/********* ESC CALIBRATION *********
  Steps (you will be prompted):
   1) DISCONNECT LiPo
   2) Arduino sets MAX -> NOW CONNECT LiPo (ESC beeps for MAX)
   3) After delay, Arduino sets MIN (ESC stores endpoints)
*/
void doEscCalibration() {
  Serial.println(F("\n=== ESC CALIBRATION START ==="));
  Serial.println(F("1) DISCONNECT LiPo from ESC."));
  Serial.println(F("2) Ready? Type 'y' + ENTER."));
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read(); // flush

  Serial.println(F("Sending MAX. NOW CONNECT LiPo. ESC should enter calibration (beeps)."));
  esc.writeMicroseconds(ESC_MAX_US);
  delay(3000);

  Serial.println(F("Sending MIN to store endpoints..."));
  esc.writeMicroseconds(ESC_MIN_US);
  delay(3000);

  Serial.println(F("Calibration done. Going to IDLE..."));
  esc.writeMicroseconds(ESC_IDLE_US);
  delay(1000);
  Serial.println(F("=== ESC CALIBRATION COMPLETE ===\n"));
}

/********* RC LEARN (Saves to EEPROM) *********
  Prompts you to hold LOW then HIGH throttle and records each.
*/
void doRcLearn() {
  Serial.println(F("\n=== RC THROTTLE LEARN START ==="));
  Serial.println(F("Make sure receiver is ON and sending CH3 to D2."));
  // Find a valid pulse first
  Serial.println(F("Looking for RC signal..."));
  while (true) {
    int p = readPulseUs(PIN_RC_THROTTLE);
    if (p >= 900 && p <= 2100) {
      Serial.print(F("RC detected: ")); Serial.print(p); Serial.println(F(" us"));
      lastSignalMs = millis();
      break;
    }
    delay(50);
  }

  // LOW capture
  Serial.println(F("Move throttle to LOW and keep it there, then press ENTER in Serial Monitor..."));
  while (!Serial.available()) {
    int p = readPulseUs(PIN_RC_THROTTLE);
    if (p >= 900 && p <= 2100) lastSignalMs = millis();
    if (millis() - lastSignalMs > FAILSAFE_MS) Serial.println(F("[WARN] RC lost..."));
    delay(20);
  }
  while (Serial.available()) Serial.read(); // flush
  // Sample a few readings
  long lowSum = 0; int lowCnt = 0;
  for (int i=0; i<50; i++) {
    int p = readPulseUs(PIN_RC_THROTTLE);
    if (p >= 900 && p <= 2100) { lowSum += p; lowCnt++; }
    delay(5);
  }
  int lowVal = (lowCnt > 0) ? (int)(lowSum/lowCnt) : 1000;

  // HIGH capture
  Serial.println(F("Now move throttle to HIGH and keep it there, then press ENTER..."));
  while (!Serial.available()) {
    int p = readPulseUs(PIN_RC_THROTTLE);
    if (p >= 900 && p <= 2100) lastSignalMs = millis();
    delay(20);
  }
  while (Serial.available()) Serial.read(); // flush
  long highSum = 0; int highCnt = 0;
  for (int i=0; i<50; i++) {
    int p = readPulseUs(PIN_RC_THROTTLE);
    if (p >= 900 && p <= 2100) { highSum += p; highCnt++; }
    delay(5);
  }
  int highVal = (highCnt > 0) ? (int)(highSum/highCnt) : 2000;

  // Sanity & store
  if (highVal - lowVal < 200) {
    Serial.println(F("[WARN] RC range too narrow. Check endpoints/subtrim on your radio."));
  }
  rcLowUs  = clamp(lowVal,  900, 1500);
  rcHighUs = clamp(highVal, 1500, 2100);
  saveRcRangeToEEPROM(rcLowUs, rcHighUs);

  Serial.print(F("Saved RC LOW = "));  Serial.print(rcLowUs);  Serial.println(F(" us"));
  Serial.print(F("Saved RC HIGH = ")); Serial.print(rcHighUs); Serial.println(F(" us"));
  Serial.println(F("=== RC THROTTLE LEARN COMPLETE ===\n"));
}

/********* NORMAL RUN *********/
void normalRunStep() {
  unsigned long now = millis();
  int thrUs = readPulseUs(PIN_RC_THROTTLE);
  bool thrValid = (thrUs >= 900 && thrUs <= 2100);
  if (thrValid) lastSignalMs = now;

  // Failsafe
  if (now - lastSignalMs > FAILSAFE_MS) {
    esc.writeMicroseconds(ESC_MIN_US);
    if (now - lastPrintMs >= PRINT_MS) {
      Serial.println(F("[FAILSAFE] RC lost → MIN"));
      lastPrintMs = now;
    }
    return;
  }

  // Map RC throttle range -> ESC range
  int target = ESC_IDLE_US; // default
  if (thrValid) {
    long m = mapLong(thrUs, rcLowUs, rcHighUs, ESC_MIN_US, ESC_MAX_US);
    target = clamp((int)m, ESC_MIN_US, ESC_MAX_US);
  }

  // Smooth
  filteredUs = (int)((1.0f - SMOOTH_ALPHA) * filteredUs + SMOOTH_ALPHA * target);
  filteredUs = clamp(filteredUs, ESC_MIN_US, ESC_MAX_US);

  esc.writeMicroseconds(filteredUs);

  if (now - lastPrintMs >= PRINT_MS) {
    Serial.print(F("RC=")); Serial.print(thrUs);
    Serial.print(F(" us  map-> ")); Serial.print(target);
    Serial.print(F(" us  ESC=")); Serial.print(filteredUs);
    Serial.print(F(" us  (rcLow=")); Serial.print(rcLowUs);
    Serial.print(F(", rcHigh=")); Serial.print(rcHighUs); Serial.println(F(")"));
    lastPrintMs = now;
  }
}

/********* SPIN TEST *********/
void spinTest2s() {
  Serial.print(F("[Spin Test] ")); Serial.print(TEST_US); Serial.println(F(" us for 2s"));
  esc.writeMicroseconds(TEST_US);
  delay(2000);
  esc.writeMicroseconds(ESC_IDLE_US);
  Serial.println(F("[Spin Test] Done."));
}

/********* ARDUINO *********/
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println(F("\n=== One-Motor Calibrate & Run ==="));
  Serial.println(F("Commands: 'c'=ESC calib, 'l'=learn RC, 'n'=normal, 's'=spin test"));
  Serial.println(F("Wiring: RC CH3->D2, ESC->D9. Props OFF. Common GND.\n"));

  esc.attach(PIN_ESC_SIGNAL);
  esc.writeMicroseconds(ESC_MIN_US);

  loadRcRangeFromEEPROM();
  Serial.print(F("EEPROM RC range: low=")); Serial.print(rcLowUs);
  Serial.print(F(" us, high=")); Serial.print(rcHighUs); Serial.println(F(" us"));
  Serial.println(F("Press 'l' to learn RC if these look wrong."));
}

void loop() {
  // Command handler (non-blocking)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C')      doEscCalibration();
    else if (c == 'l' || c == 'L') doRcLearn();
    else if (c == 's' || c == 'S') spinTest2s();
    else if (c == 'n' || c == 'N') Serial.println(F("Normal run active (press 'l' to relearn)."));
  }

  // Normal run always active; if RC not learned, it still uses defaults
  normalRunStep();
  delay(3);
}
