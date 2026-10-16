// HC-SR04 proximity sensor for the truck dashboard (ESP32).
//
// Wiring:  HC-SR04 VCC -> 5V (VIN)   GND -> GND
//          TRIG -> GPIO 5
//          ECHO -> GPIO 18 through a divider (ECHO -> 1k -> GPIO18 -> 2k -> GND),
//                  because ECHO is a 5 V signal and ESP32 pins are 3.3 V.
//
// Prints one line per reading at ~20 Hz:  "D <cm>"  (-1 = no echo / out of range)
// The dashboard server reads these over USB serial (115200 baud).

const int TRIG = 5;
const int ECHO = 18;
const unsigned long TIMEOUT_US = 25000;   // ~4.3 m round trip; beyond that report -1

long readOnce() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, TIMEOUT_US);
  if (us == 0) return -1;
  return (long)(us / 58);                 // microseconds -> centimetres
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
}

void loop() {
  // Median of three readings suppresses the sensor's occasional wild value.
  long a = readOnce(); delay(12);
  long b = readOnce(); delay(12);
  long c = readOnce();
  long lo = min(a, min(b, c)), hi = max(a, max(b, c));
  long med = a + b + c - lo - hi;
  // If any reading timed out, trust a real echo over -1 so a person is never missed.
  if (med < 0) med = hi;
  Serial.print("D "); Serial.println(med);
  delay(12);
}
