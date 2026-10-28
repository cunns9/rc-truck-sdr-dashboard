// Truck camera for the dashboard (AI-Thinker ESP32-CAM on an ESP32-CAM-MB).
//
// The board makes its own WiFi network, so no lab WiFi login is needed:
//   SSID "TruckCam", password "truckcam2026", board address 192.168.4.1
// Join it from the Mac (Ethernet stays connected), then open:
//   http://192.168.4.1/          page with the live view
//   http://192.168.4.1:81/stream MJPEG stream (for the dashboard)
//   http://192.168.4.1/capture   one JPEG (for person detection)
//
// USB mode (no WiFi needed, bench only): the Mac sends "f" and gets back
// "JPG" + 4-byte little-endian length + the JPEG. "q" switches to 320x240, "v" to
// 640x480, "b" + 4-byte little-endian baud changes the serial speed (boots at 115200).
// server.py uses this (Camera class).
//
// Flash: arduino-cli compile/upload --fqbn esp32:esp32:esp32cam
// The MB enters upload mode by itself; no buttons needed.

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>

const char *AP_SSID = "TruckCam";
const char *AP_PASS = "truckcam2026";
const int AP_CHANNEL = 6;   // 2.4 GHz overlaps the truck remote; change if driving glitches
const unsigned long USB_BAUD = 115200;

// AI-Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define FLASH_LED       4

static const char *BOUNDARY = "frame";

static const char PAGE[] =
  "<!doctype html><html><head><meta name=viewport content='width=device-width'>"
  "<title>TruckCam</title><style>body{margin:0;background:#111;color:#ddd;font-family:sans-serif;text-align:center}"
  "img{max-width:100%;}</style></head><body><h3>TruckCam</h3>"
  "<img id=v><script>v.src='http://'+location.hostname+':81/stream'</script></body></html>";

esp_err_t indexHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return r;
}

esp_err_t streamHandler(httpd_req_t *req) {
  char part[64];
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    int n = snprintf(part, sizeof(part),
                     "\r\n--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     BOUNDARY, fb->len);
    esp_err_t r = httpd_resp_send_chunk(req, part, n);
    if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (r != ESP_OK) return r;   // viewer closed the page
  }
}

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  httpd_handle_t web = NULL, stream = NULL;
  httpd_uri_t index = {"/", HTTP_GET, indexHandler, NULL};
  httpd_uri_t capture = {"/capture", HTTP_GET, captureHandler, NULL};
  httpd_uri_t streamUri = {"/stream", HTTP_GET, streamHandler, NULL};
  if (httpd_start(&web, &cfg) == ESP_OK) {
    httpd_register_uri_handler(web, &index);
    httpd_register_uri_handler(web, &capture);
  }
  // The stream never returns, so it gets its own server on port 81.
  cfg.server_port = 81;
  cfg.ctrl_port += 1;
  if (httpd_start(&stream, &cfg) == ESP_OK) httpd_register_uri_handler(stream, &streamUri);
}

void setup() {
  Serial.begin(USB_BAUD);
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.grab_mode = CAMERA_GRAB_LATEST;   // always send the newest frame, not a stale one
  if (psramFound()) {
    c.frame_size = FRAMESIZE_VGA;     // 640x480
    c.jpeg_quality = 12;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.frame_size = FRAMESIZE_QVGA;    // 320x240
    c.jpeg_quality = 15;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
  }
  esp_err_t err = esp_camera_init(&c);
  Serial.printf("PSRAM %s, camera init %s\n", psramFound() ? "yes" : "no",
                err == ESP_OK ? "OK" : esp_err_to_name(err));
  if (err != ESP_OK) return;
  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("camera sensor PID 0x%02x\n", s->id.PID);

  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL);
  Serial.printf("WiFi \"%s\" up, open http://%s/\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  startServers();
}

void sendFrameUsb() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  uint32_t n = fb->len;
  Serial.write((const uint8_t *)"JPG", 3);
  Serial.write((const uint8_t *)&n, 4);   // ESP32 is little-endian
  Serial.write(fb->buf, n);
  esp_camera_fb_return(fb);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    sensor_t *s = esp_camera_sensor_get();
    if (c == 'f') sendFrameUsb();
    else if (c == 'q' && s) s->set_framesize(s, FRAMESIZE_QVGA);
    else if (c == 'v' && s) s->set_framesize(s, FRAMESIZE_VGA);
    else if (c == 'b') {
      uint32_t baud = 0;
      if (Serial.readBytes((char *)&baud, 4) == 4 && baud >= 9600 && baud <= 3000000) {
        Serial.flush();
        Serial.updateBaudRate(baud);
      }
    }
  }
  delay(1);
}
