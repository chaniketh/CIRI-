/*
  Standalone Arduino Uno load-cell calibration.
  Library: HX711 by Bogdan Necula / bogde.
  HX711 VCC=5V, GND=GND, DT/DOUT=D2, SCK=D3; bridge on E+/E-/A+/A-.
  Serial Monitor: 115200 baud, Newline or Both NL & CR.

  t       -> tare with no applied load
  c 50    -> calibrate with ACTUAL total hanging mass of 50 grams
  v 100   -> verify with a DIFFERENT actual mass of 100 grams
  p       -> print calibration constants (only after verification passes)
  r       -> reset calibration
  h       -> help

  Keep cart secured. Apply every load at the SAME needle contact point and
  direction. A mass gives m*g only when its force reaches that point (for
  example freely hanging along an upright rail or through a suitable pulley).
  This calibrates response to that applied force, not unknown multiaxis forces.
  Uploading this replaces the PC logger's CSV firmware. Re-upload
  uno_load_cell.ino before using run_test.py; this sketch does NOT create its
  calibration.json. Constants here are session-only, not saved in EEPROM.
*/
#include <HX711.h>
#include <math.h>
#include <stdlib.h>

HX711 cell;
const byte SAMPLE_COUNT = 30;
float zeroCounts = 0;
float zeroNoise = 0;
float countsPerGram = 0;
float calibrationMass = 0;
bool tared = false;
bool calibrated = false;
bool verified = false;
char command[40];
byte commandLength = 0;
bool commandOverflow = false;
unsigned long lastPrint = 0;
unsigned long lastReady = 0;
unsigned long lastFault = 0;

void help() {
  Serial.println(F("Commands: t | c <grams> | v <different grams> | p | r | h"));
  Serial.println(F("1) Unload, settle, send t. 2) Apply known mass, settle, send c 50."));
  Serial.println(F("3) Replace with a different known mass, settle, send v 100."));
  Serial.println(F("4) If verification passes, send p to print constants."));
  Serial.println(F("Use your ACTUAL total masses, including any hanger. Settings: 115200 + Newline."));
}

bool capture(float &mean, float &noise) {
  mean = 0;
  float sumSquared = 0;
  unsigned long start = millis();
  Serial.println(F("Sampling 30 conversions; keep everything still..."));
  for (byte i = 1; i <= SAMPLE_COUNT; ++i) {
    while (!cell.is_ready()) {
      if (millis() - start > 6000UL) {
        Serial.println(F("ERROR: HX711 timeout. Check 5V/GND, DT=D2, SCK=D3."));
        return false;
      }
      delay(1);
    }
    long raw = cell.read();
    lastReady = millis();
    if (raw <= -8388607L || raw >= 8388606L) {
      Serial.println(F("ERROR: ADC saturated. Check bridge wiring/load."));
      return false;
    }
    float delta = raw - mean;
    mean += delta / i;
    sumSquared += delta * (raw - mean);
  }
  noise = sqrt(sumSquared / (SAMPLE_COUNT - 1));
  Serial.print(F("Mean counts=")); Serial.print(mean, 2);
  Serial.print(F("  noise stddev=")); Serial.println(noise, 2);
  return true;
}

void printConstants() {
  if (!verified) {
    Serial.println(F("Not verified. Complete t, c <grams>, then v <different grams>."));
    return;
  }
  Serial.println(F("VERIFIED session calibration (check return to zero after unloading)."));
  Serial.print(F("OFFSET_COUNTS = ")); Serial.println(zeroCounts, 2);
  Serial.print(F("COUNTS_PER_GRAM = ")); Serial.println(countsPerGram, 6);
  Serial.print(F("COUNTS_PER_NEWTON = ")); Serial.println(countsPerGram / 0.00980665f, 6);
  Serial.println(F("Force_N = (raw_counts - OFFSET_COUNTS) / COUNTS_PER_NEWTON"));
  Serial.println(F("HX711 get_units(): use set_scale(COUNTS_PER_GRAM) for grams-equivalent."));
  Serial.println(F("Save these values yourself. Re-tare in the final test orientation."));
}

void processCommand() {
  if (commandOverflow) {
    Serial.println(F("Command too long; discarded."));
    return;
  }
  command[commandLength] = '\0';
  char op = command[0];
  if (op >= 'A' && op <= 'Z') op += 'a' - 'A';
  if (op == 'h') { help(); return; }
  if (op == 'r') {
    tared = calibrated = verified = false;
    Serial.println(F("Reset. Unload and send t."));
    return;
  }
  if (op == 'p') { printConstants(); return; }
  if (op == 't') {
    calibrated = verified = false;
    tared = false;
    if (capture(zeroCounts, zeroNoise)) {
      tared = true;
      Serial.println(F("Tared. Apply known mass at needle contact, settle, send c <grams>."));
    }
    return;
  }
  if (op != 'c' && op != 'v') { help(); return; }
  if (!tared) { Serial.println(F("First remove load and send t.")); return; }
  char *end;
  float grams = strtod(command + 1, &end);
  while (*end == ' ' || *end == '\t') ++end;
  if (end == command + 1 || *end != '\0' || !isfinite(grams) || grams <= 0) {
    Serial.println(F("Enter a positive known mass in grams, e.g. c 50 or v 100."));
    return;
  }
  if (op == 'v' && !calibrated) {
    Serial.println(F("Calibrate first with c <grams>.")); return;
  }
  if (op == 'v' && fabs(grams - calibrationMass) < 0.2f * calibrationMass) {
    Serial.println(F("Verify using a mass at least 20% different from calibration mass.")); return;
  }
  float mean, noise;
  if (op == 'c') calibrated = verified = false;
  else verified = false;
  if (!capture(mean, noise)) return;
  if (op == 'c') {
    float change = mean - zeroCounts;
    float combinedNoise = sqrt(noise * noise + zeroNoise * zeroNoise);
    if (fabs(change) < 1 || fabs(change) < 10 * combinedNoise ||
        combinedNoise > 0.05f * fabs(change)) {
      Serial.println(F("REJECTED: too little response or unstable load. Check mounting/wiring."));
      return;
    }
    countsPerGram = change / grams; // Negative slopes are valid.
    calibrationMass = grams;
    calibrated = true;
    Serial.print(F("Candidate counts/gram=")); Serial.println(countsPerGram, 6);
    Serial.println(F("Now apply a DIFFERENT known mass and send v <grams>."));
  } else {
    float measured = (mean - zeroCounts) / countsPerGram;
    float error = measured - grams;
    float noiseGrams = sqrt(noise * noise + zeroNoise * zeroNoise) / fabs(countsPerGram);
    Serial.print(F("Expected g=")); Serial.print(grams, 3);
    Serial.print(F("  measured g=")); Serial.print(measured, 3);
    Serial.print(F("  error g=")); Serial.print(error, 3);
    Serial.print(F("  noise g=")); Serial.println(noiseGrams, 3);
    if (fabs(error) > 0.05f * grams || noiseGrams > 0.05f * grams) {
      Serial.println(F("REJECTED: verification error/noise exceeds 5%. Do not use this factor."));
      calibrated = false;
      return;
    }
    verified = true;
    printConstants();
    Serial.println(F("Remove mass: live grams-equivalent should return close to zero."));
    Serial.println(F("Passing is a coarse check, not a guarantee of measurement accuracy."));
  }
}

void setup() {
  Serial.begin(115200);
  cell.begin(2, 3, 128);
  lastReady = millis();
  Serial.println(F("UNO HX711 CALIBRATION -- no motor control"));
  help();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (commandLength || commandOverflow) processCommand();
      commandLength = 0;
      commandOverflow = false;
    } else if (commandLength < sizeof(command) - 1) {
      command[commandLength++] = ch;
    } else commandOverflow = true;
  }
  if (cell.is_ready()) {
    long raw = cell.read();
    lastReady = millis();
    if (millis() - lastPrint >= 500UL) {
      lastPrint = millis();
      Serial.print(F("raw=")); Serial.print(raw);
      if (calibrated && raw > -8388607L && raw < 8388606L) {
        float grams = (raw - zeroCounts) / countsPerGram;
        Serial.print(verified ? F("  verified g_equiv=") : F("  UNVERIFIED g_equiv="));
        Serial.print(grams, 3);
        Serial.print(F("  force_N=")); Serial.print(grams * 0.00980665f, 4);
      }
      if (raw <= -8388607L || raw >= 8388606L) {
        Serial.print(F("  ERROR: ADC SATURATION"));
      }
      Serial.println();
    }
  } else if (millis() - lastReady > 1000UL && millis() - lastFault > 1000UL) {
    lastFault = millis();
    Serial.println(F("ERROR: HX711_NOT_READY; check power, DT=D2, SCK=D3."));
  }
}
