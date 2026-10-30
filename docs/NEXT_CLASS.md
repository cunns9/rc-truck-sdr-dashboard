# Next class: start here

Where we stopped (2026-10-30): the camera safety stop works on the real truck. Driving from
the dashboard through the HackRF, the truck drives with nobody in view and holds (sends
idle) while a person is in view. The distance-sensor stop was tested on 2026-10-28.
Full history: [PROGRESS.md](PROGRESS.md).

## 1. Setup (5 min)
1. Plug into the USB hub: **HackRF**, **Elegoo Mega** (HC-SR04 on Trig D9 / Echo D10,
   VCC 5V, GND), **ESP32-CAM on its MB board** (micro-USB data cable).
2. Check GitHub is your account before pushing: `gh api user -q .login` must print `cunns9`.
   If not: `gh auth login --web` (other people's accounts are also on this Mac).
   **Last class's work is committed locally but NOT pushed yet**: push it first (section 4).
3. Start the dashboard from this folder:
   `cd ~/Desktop/SDR/dashboard && python3 server.py`, then open http://localhost:8642
4. Check the dashboard shows: Person distance, Truck camera (live picture), the
   "Living in view" line, and both **Safety stop** and **Camera stop** ticked.
5. Truck: switch it on and drive it **from the dashboard** (truck controls are on 2415 MHz).
   Keep the physical remote off: it bypasses the safety stop.

If the camera picture freezes, reload the page. If the camera port is missing, replug the
MB; it shows as `/dev/cu.usbserial-*`.

## 2. Today's goal: TFmini-S LiDAR on the ESP32-CAM
This lets the camera and the distance sensor ride on the truck on one board, replacing the Mega.

Wiring (check the colours against the TFmini-S label; its signals are 3.3 V, no resistors):

| TFmini-S wire | ESP32-CAM pin |
|---|---|
| Red (+5V) | 5V |
| Black (GND) | GND |
| Green (TX, sends the distance) | GPIO 13 |
| White (RX) | GPIO 14 |

GPIO 13 / 14 are free because the SD card isn't used. Avoid GPIO 0, 2, 12, 15 (boot pins)
and GPIO 4 (flash LED).

Steps:
1. `truckcam.ino`: read the TFmini-S (115200 baud, 9-byte frames starting 0x59 0x59,
   distance in cm) on GPIO 13 and send it to the Mac along with the camera frames.
2. `server.py`: feed that distance into the Person distance tile and the safety stop
   (`Proximity`), so the Mega isn't needed.
3. Check the reading against a tape measure at about 1, 2 and 3 ft, then do one hold test
   on the truck.

## 3. Still open
- **WiFi for the camera:** the Mac can't join `TruckCam` without losing internet. Bring/ask
  for an Ethernet cable, or get the lab WiFi login from the professor.
- **Battery for the truck:** USB power bank into the MB's micro-USB.
- **2.4 GHz interference:** camera WiFi overlaps the truck's channels; test before relying on it.
- Excavator and tractor controls not captured yet.
- Camera stop fails open (frozen camera = no hold), same as the distance sensor. Revisit
  before relying on it with people around.

## 4. Before leaving
- `pkill -f server.py`
- Commit and push (HTTPS, as cunns9):
  `git -c credential.helper= -c credential.helper='!gh auth git-credential' push origin HEAD:main`
- Log your GitHub out of this shared Mac: `gh auth logout -u cunns9`
