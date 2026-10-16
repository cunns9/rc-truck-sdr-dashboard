// HC-SR04 proximity sensor for the truck dashboard (ESP32, WiFi).
// Rides on the truck; posts distance to the dashboard over WiFi. No USB after flashing.
//
// Wiring:  HC-SR04 VCC -> 5V (VIN)   GND -> GND   TRIG -> GPIO 5
//          ECHO -> 1k -> GPIO 18 -> 2k -> GND   (ECHO is 5 V, ESP32 pins are 3.3 V)
// Power:   any USB battery pack / 5 V source into the ESP32's USB or VIN.
//
// Fill in the three settings below, flash, then watch the "Person distance" tile.

#include <WiFi.h>
#include <HTTPClient.h>

const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* DASHBOARD = "http://10.178.123.33:8642/api/proximity";   // the Mac's IP

const int TRIG = 5, ECHO = 18;
const unsigned long TIMEOUT_US = 25000;

long readOnce() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, TIMEOUT_US);
  return us == 0 ? -1 : (long)(us / 58);
}

long readMedian() {
  long a = readOnce(); delay(12);
  long b = readOnce(); delay(12);
  long c = readOnce();
  long lo = min(a, min(b, c)), hi = max(a, max(b, c));
  long med = a + b + c - lo - hi;
  return med < 0 ? hi : med;       // never hide a real echo behind a timeout
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print("."); }
  Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : " failed");
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT); digitalWrite(TRIG, LOW);
  WiFi.mode(WIFI_STA);
  connectWifi();
}

void loop() {
  long cm = readMedian();
  Serial.print("D "); Serial.println(cm);          // also works over USB if tethered
  connectWifi();
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.setConnectTimeout(300);
    http.setTimeout(300);
    http.begin(DASHBOARD);
    http.addHeader("Content-Type", "application/json");
    http.POST("{\"cm\":" + String(cm) + ",\"source\":\"wifi\"}");
    http.end();
  }
  delay(20);                                        // ~15-20 readings per second
}
