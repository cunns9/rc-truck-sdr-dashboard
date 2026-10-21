// Diagnostic for HC-SR04 wiring on ESP32-S3.
// 1) Scans safe GPIOs with internal pull-up: a powered HC-SR04 drives ECHO low,
//    so the pin ECHO is actually plugged into shows up as "LOW".
// 2) Tries TRIG=5/ECHO=18 and the swapped pair.
const int SAFE[] = {1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21,38,39,40,41,42,47,48};
void scan() {
  Serial.print("pulled-low pins:");
  for (int p : SAFE) {
    pinMode(p, INPUT_PULLUP); delayMicroseconds(200);
    if (digitalRead(p) == 0) Serial.printf(" %d", p);
    pinMode(p, INPUT);
  }
  Serial.println();
}
void tryPair(int trig, int echo) {
  pinMode(trig, OUTPUT); pinMode(echo, INPUT_PULLUP);
  digitalWrite(trig, LOW); delayMicroseconds(5);
  int idle = digitalRead(echo);
  digitalWrite(trig, HIGH); delayMicroseconds(10); digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 30000);
  Serial.printf("trig=%d echo=%d idle(pullup)=%d pulse_us=%lu cm=%ld\n",
                trig, echo, idle, us, us ? (long)(us / 58) : -1);
  pinMode(trig, INPUT); pinMode(echo, INPUT);
}
void setup() { Serial.begin(115200); delay(500); Serial.println("DIAG2 start"); }
void loop() {
  scan();
  tryPair(5, 18); delay(60);
  tryPair(18, 5); delay(60);
  Serial.println("--"); delay(700);
}
