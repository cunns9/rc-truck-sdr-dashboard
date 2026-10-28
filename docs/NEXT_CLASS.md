# Next class: start here

Where we stopped (2026-10-28): the distance sensor works on the Elegoo Mega, the ESP32-CAM
streams to the dashboard over USB, and YOLO detection labels people/animals (living) vs
objects on the live view. Full history: [PROGRESS.md](PROGRESS.md).

## 1. Setup (5 min)
1. Plug into the USB hub: **HackRF**, **Elegoo Mega** (HC-SR04 on Trig D9 / Echo D10,
   VCC 5V, GND), **ESP32-CAM on its MB board** (micro-USB data cable).
   The Logitech C920 is only needed for documentation photos.
2. Check GitHub is your account before pushing: `gh api user -q .login` must print `cunns9`.
   If not: `gh auth login --web` (other people's accounts are also on this Mac).
3. Start the dashboard from this folder:
   `cd ~/Desktop/SDR/dashboard && python3 server.py`, then open http://localhost:8642
4. Check the dashboard shows: Person distance (cm reading), Truck camera (live picture,
   ~6-14 frames/s), and a "Living in view / No people or animals in view" line.

If the camera picture freezes, reload the page. If the camera port is missing, replug the
MB; it shows as `/dev/cu.usbserial-*`.

## 2. Today's goals, in order
1. **Verify detection on a whole person.** Put the camera on the desk 1-2 m away at eye
   level (head and shoulders in frame). Expect a red "person NN%" box. Also try a bottle
   or cup (grey box). If the picture is upside down, flip it in `truckcam.ino`
   (`s->set_vflip(s, 1); s->set_hmirror(s, 1);`).
2. **Camera feeds the safety stop.** When something living is in view, hold the truck the
   same way the distance sensor does (`Radio.guarded()` in `server.py`), with its own
   on/off toggle on the dashboard.
3. **First real safety-stop test** with the HackRF: save/choose a truck control, have
   someone stand within 5 ft, press drive, and confirm the truck gets idle instead.
   Watch the "Stopped N times" counter.
4. **Wire the TFmini-S LiDAR to the ESP32-CAM** (5V/GND from the CAM, TX/RX to two free
   GPIOs) and send its distance alongside the video. This replaces the Mega on the truck.

## 3. Still open
- **WiFi for the camera:** the Mac has no Ethernet right now, so it can't join the
  `TruckCam` network without losing internet. Bring/ask for an Ethernet cable, or get the
  lab WiFi login from the professor. Phone test: join `TruckCam` (pw `truckcam2026`) and
  open http://192.168.4.1/.
- **Battery for the truck:** USB power bank into the MB's micro-USB.
- **2.4 GHz interference:** camera WiFi overlaps the truck's channels; test before relying
  on it.
- Excavator and tractor controls not captured yet.
- Recordings in `docs/recordings/` are local only; decide whether to commit them.

## 4. Before leaving
- `pkill -f server.py`
- Commit and push (HTTPS, as cunns9):
  `git -c credential.helper= -c credential.helper='!gh auth git-credential' push origin HEAD:main`
- Log your GitHub out of this shared Mac: `gh auth logout -u cunns9`
