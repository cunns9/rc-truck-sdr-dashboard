// Full pair sweep: pulse each candidate as TRIG, listen on each other as ECHO.
const int P[] = {1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21,38,39,40,41,42,47,48};
const int N = sizeof(P)/sizeof(P[0]);
void setup() { Serial.begin(115200); delay(500); Serial.println("DIAG5 start");
  for (int i=0;i<N;i++) pinMode(P[i], INPUT); }
void loop() {
  Serial.print("pulled-low pins:");
  for (int i=0;i<N;i++){ pinMode(P[i],INPUT_PULLUP); delayMicroseconds(200);
    if(!digitalRead(P[i])) Serial.printf(" %d",P[i]); pinMode(P[i],INPUT);} Serial.println();
  int hits=0;
  for (int i=0;i<N;i++) {
    int t=P[i]; pinMode(t,OUTPUT); digitalWrite(t,LOW);
    for (int j=0;j<N;j++) { if(i==j) continue; int e=P[j];
      delayMicroseconds(5); digitalWrite(t,HIGH); delayMicroseconds(10); digitalWrite(t,LOW);
      unsigned long us=pulseIn(e,HIGH,25000);
      if(us>100){ Serial.printf("HIT trig=%d echo=%d %lu us = %ld cm\n",t,e,us,(long)(us/58)); hits++; }
      delay(3);
    }
    pinMode(t,INPUT);
  }
  Serial.printf("-- sweep done, %d hits\n", hits); delay(500);
}
