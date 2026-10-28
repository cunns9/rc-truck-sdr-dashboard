# RC truck project: progress log

HackRF dashboard that drives a 2.4 GHz RC construction set (truck, loader, excavator,
tractor) from the Mac, plus a person-proximity safety stop and an on-truck camera.

Run the dashboard: `python3 server.py` then open http://localhost:8642
(stop it with `pkill -f server.py`).

## Hardware

| Part | Role | Status |
|---|---|---|
| HackRF One | Transmits remote packets to the vehicles | Working |
| Elegoo Mega 2560 + HC-SR04 | Desk distance sensor over USB | **Working** (2026-10-28) |
| ESP32-CAM (AI-Thinker, OV2640) on ESP32-CAM-MB | Truck camera, own WiFi network | **Flashed and streaming** (2026-10-28) |
| Benewake TFmini-S LiDAR | Truck distance sensor (0.1-12 m, UART) | Found, not wired yet |
| Logitech C920 | Desk webcam for documentation photos/video | Working, plug in as needed |
| ESP32-S3 DevKitC-1 (N16R8) | Earlier sensor board | Retired: HC-SR04 never echoed on it |

## 2026-10-28

### Distance sensor works on the Mega
- The HC-SR04 gave no echo at all on the 3.3 V ESP32-S3 (sessions 10-21 and 10-23).
  Moving it to a 5 V Elegoo Mega 2560 fixed it on the first try.
- Wiring (direct, no resistors): VCC -> 5V, GND -> GND, Trig -> D9, Echo -> D10.
- `sensor/sensor.ino` builds for both boards (`#ifdef ARDUINO_AVR_MEGA2560` picks the pins).
  Flash: `arduino-cli upload -p /dev/cu.usbmodem* --fqbn arduino:avr:mega sensor`
  (stop `server.py` first, it holds the port).
- Dashboard "Person distance" tile reads it live; the safety stop reports *blocked*
  under the 152 cm (5 ft) threshold. Not yet tested against a real truck transmission.

### Truck camera
- Goal: camera + distance sensor on the truck, battery powered, video on the dashboard
  while driving.
- Parts found in the lab: ESP32-CAM, its ESP32-CAM-MB USB base, TFmini-S LiDAR.
  Planned truck kit: ESP32-CAM + TFmini-S + USB power bank into the MB's micro-USB
  (the TFmini-S takes 5V/GND from the ESP32-CAM). Don't power the camera from the
  Mega's 5V pin: WiFi bursts brown it out.
- ESP32-CAM checks: CH340 port `/dev/cu.usbserial-*`, ESP32-D0WD-V3, 4 MB flash, PSRAM,
  OV2640 sensor. The MB enters upload mode by itself (no buttons).
- `truckcam/truckcam.ino`: the board creates WiFi network **TruckCam**
  (channel 6, address 192.168.4.1).
  - `http://192.168.4.1/` live view page
  - `http://192.168.4.1:81/stream` MJPEG stream (for the dashboard)
  - `http://192.168.4.1/capture` single JPEG (for person detection)
  - Flash: `arduino-cli upload -p /dev/cu.usbserial-* --fqbn esp32:esp32:esp32cam truckcam`

### Documentation media
- C920 recordings and photos are kept in `docs/recordings/` (local only, git-ignored
  via `.git/info/exclude`; decide later whether to commit them). Video only, no audio.

## Open issues / next steps
1. **Mac networking:** the Mac is on the lab WiFi (Ethernet inactive). Joining TruckCam
   would cut its internet. Options: plug Ethernet back in (preferred), put the camera on
   the lab WiFi (needs credentials from the professor), or add a USB WiFi adapter.
   Meanwhile, view the stream from a phone.
2. Embed the camera stream in the dashboard and add person detection (YOLO) on the Mac.
3. Wire the TFmini-S to the ESP32-CAM and send its distance to `/api/proximity`.
4. **Radio interference:** the truck remote uses 2415/2435/2470 MHz, which overlaps every
   WiFi channel. Test driving with the stream running; lower the resolution or move the
   channel if commands drop.
5. First real safety-stop test: transmit a drive command with someone inside 5 ft.
6. Excavator and tractor: no controls captured yet.
