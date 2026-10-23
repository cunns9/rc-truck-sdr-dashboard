// Fire TRIG on 4, then sample pin 5 with the ADC for 40 ms and report the peak voltage.
void setup(){ Serial.begin(115200); delay(500); analogSetAttenuation(ADC_11db);
  pinMode(4, OUTPUT); digitalWrite(4, LOW); pinMode(5, INPUT); Serial.println("DIAG7 start"); }
void loop(){
  digitalWrite(4, HIGH); delayMicroseconds(10); digitalWrite(4, LOW);
  unsigned long t0 = micros(); int peak = 0, n = 0; unsigned long tpk = 0, above = 0;
  while (micros() - t0 < 40000) {
    int v = analogReadMilliVolts(5); n++;
    if (v > peak) { peak = v; tpk = micros() - t0; }
    if (v > 500) above++;
  }
  Serial.printf("peak=%d mV at %lu us, samples=%d, samples>0.5V=%lu\n", peak, tpk, n, above);
  delay(500);
}
