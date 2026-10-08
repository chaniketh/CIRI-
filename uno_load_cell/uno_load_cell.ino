// Arduino Uno + bogde HX711 library. All calibration is done by the PC logger.
// HX711 VCC -> 5V, GND -> GND, DT/DOUT -> D2, SCK -> D3.
#include <HX711.h>
HX711 cell;
uint32_t sequence = 0;
unsigned long lastSample = 0;
unsigned long lastFaultReport = 0;
void setup() {
  Serial.begin(115200);
  cell.begin(2, 3, 128); // Channel A, gain 128
  lastSample = millis();
  Serial.println(F("# needle-load-cell-v1: sequence,uno_ms,raw_counts,saturated"));
}
void loop() {
  // One output for each actual conversion; no averaging that hides short peaks.
  if (cell.is_ready()) {
    long raw = cell.read();
    lastSample = millis();
    bool saturated = raw <= -8388607L || raw >= 8388606L;
    Serial.print(sequence++); Serial.print(',');
    Serial.print(lastSample); Serial.print(',');
    Serial.print(raw); Serial.print(','); Serial.println(saturated ? 1 : 0);
  } else if (millis() - lastSample > 1000UL && millis() - lastFaultReport > 1000UL) {
    Serial.println(F("# ERROR HX711_NOT_READY"));
    lastFaultReport = millis();
  }
}
