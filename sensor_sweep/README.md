# HC-SR04 diagnostics (2026-10-23)

Both compile/flash with:
    FQBN=esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc,UploadSpeed=921600
    arduino-cli compile --fqbn $FQBN --build-path /tmp/b <sketch_dir>
    arduino-cli upload -p /dev/cu.usbmodem* --fqbn $FQBN --input-dir /tmp/b <sketch_dir>
(stop server.py first; it holds the serial port)

- sensor_sweep: lists GPIOs pulled low, then pulses every candidate pin as TRIG and
  listens on every other pin for an echo. Prints "HIT trig=X echo=Y ... cm" for real
  pairs. 100-200 us hits on random pins are crosstalk, ignore them.
- sensor_probe: fires TRIG on 4, samples pin 5 with the ADC for 40 ms, prints the peak
  voltage. Distinguishes "no echo pulse at all" from "divider ratio too low".

Status at end of 2026-10-23: wiring confirmed at the ESP32 end (yellow=5V, purple=GND,
white Trig=pin 4, blue Echo=pin 5 via breadboard divider), yet zero echo in every test:
peak on pin 5 < 0.2 V after trigger. Sensor end / breadboard still unverified; next step
is the 4-wire direct test at 3V3 (VCC->3V3, Trig->4, Echo->5 direct, GND->GND) and, if
still silent, swap the HC-SR04 module.

Resolved 2026-10-28: moved the HC-SR04 to a 5 V Elegoo Mega 2560 (Trig D9, Echo D10,
direct) and it worked immediately. See docs/PROGRESS.md.
