#!/usr/bin/env python3
"""Live dashboard for the toy vehicles' 2.4 GHz remotes (truck, loader, excavator, tractor).

Streams IQ from a HackRF, finds the remote's packets, decodes them and serves
a web page at http://localhost:8642.

    python3 server.py                     # live from the HackRF
    python3 server.py --file ../cap_2410.iq --fs 20e6 --offset 5e6   # replay a capture
"""
import argparse
import collections
import json
import os
import struct
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # dashboard still works without the proximity sensor
    serial = None

HERE = os.path.dirname(os.path.abspath(__file__))
LABELS_PATH = os.path.join(HERE, "labels.json")
PORT = 8642

# Truck: 2415/2435/2470. Loader: 2418/2438/2458/2473. Excavator: 2411/2431.
# Re-pairing a remote can move it, and a remote's payload differs per channel.
CHANNELS = [2411, 2415, 2418, 2431, 2435, 2438, 2458, 2470, 2473]
VEHICLES = ["truck", "loader", "excavator", "tractor"]
BITRATE = 1e6
SYNC = "0001111110110"   # the truck remote's sync; each remote has its own (labels store it)
SYNC_BITS = 13
HEADER_BITS = 19         # raw bits between sync and the 3x-repeated payload
PAYLOAD_BITS = 112
CHUNK_SECONDS = 0.1
RATE_HISTORY = 150       # chunks kept for the packet-rate chart
PREAMBLE = "10" * 16
DEVIATION_HZ = 340e3     # measured on the real remote
FRAME_SECONDS = 0.02     # one packet group every 20 ms
COPY_SPACING = 0.0022    # three copies 2.2 ms apart
TX_GAIN = 20
STOP_CM = 183            # safety stop when someone is closer than this (6 ft; TRIAL_PLAN.md tests 1/3/5 ft inside it)
RELEASE_CM = 15          # hysteresis: resume only once this much farther away
SENSOR_STALE_S = 1.0     # no reading for this long -> treat as "sensor offline"
SENSOR_HOLD_S = 0.5      # keep holding this long after the last close reading (rides out 'no echo' gaps)
SENSOR_BAUD = 115200
CAMERA_BAUD = 460800     # fastest the Mac's CH340 driver handles reliably (921600 garbles)
CAMERA_VIDS = (0x1a86,)  # CH340 USB chip on the ESP32-CAM-MB
CAMERA_STALE_S = 2.0
DETECT_MODEL = os.path.join(HERE, "models", "yolo11n.pt")   # downloaded on first run if missing
DETECT_CONF = 0.4        # minimum YOLO confidence to count a detection
CAMERA_HOLD_S = 1.0      # camera stop stays on this long after the last living detection (rides out flicker)
LIVING = {"person", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe"}
DETECT_IMGSZ = 320       # matches the camera's 320x240 frames
CAMERA_CAL_PATH = os.path.join(HERE, "camera_cal.json")   # camera range calibration (box height at the stop distance)
CAMERA_CAL_SLACK = 0.95  # hold for boxes a little smaller than the calibrated one (errs toward stopping)
STARTED = time.time()
TX_FRAME_PATH = os.path.join(HERE, "tx_frame.iq")
EVENTS = collections.deque(maxlen=500)   # stop-test timeline, GET /api/events
EVENTS_LOG = os.path.join(HERE, "docs", "stop_events.jsonl")


def log_event(name, **info):
    """Timestamp a safety-stop event (trigger seen, transmitter switched) for stop-time tests.

    Only records; never changes what is transmitted.
    """
    t = time.time()
    e = {"t": round(t, 3), "event": name, **info}
    EVENTS.append(e)
    print(f"[event] {time.strftime('%H:%M:%S', time.localtime(t))}.{int(t % 1 * 1000):03d} {name} {info}", flush=True)
    try:
        with open(EVENTS_LOG, "a") as f:
            f.write(json.dumps(e) + "\n")
    except OSError:
        pass


def event_summary():
    """Pair each tx_idle with the trigger just before it: how long the Mac took to switch to idle."""
    out, last = [], {}
    for e in EVENTS:
        if e["event"] in ("sensor_near", "camera_person"):
            last[e["event"]] = e
        elif e["event"] == "tx_idle":
            trig = [x for x in last.values() if x["t"] <= e["t"] and e["t"] - x["t"] < 5]
            if trig:
                first = min(trig, key=lambda x: x["t"])
                row = {"at": e["t"], "trigger": first["event"], "trigger_to_idle_ms": round((e["t"] - first["t"]) * 1e3)}
                if first["event"] == "camera_person":
                    row["frame_to_idle_ms"] = round((e["t"] - first["frame_at"]) * 1e3)
                    row["infer_ms"] = first.get("infer_ms")
                if first["event"] == "sensor_near":
                    row["cm"] = first.get("cm")
                out.append(row)
            last = {}
    return out


def packet_bits(header, payload, sync=SYNC):
    return PREAMBLE + sync + header + "".join(b * 3 for b in payload) + "1010"


def synth_frame(header, payload, fs, offset_hz, sync=SYNC):
    """One 20 ms frame of GFSK (three packet copies) as int8 IQ, shifted by offset_hz."""
    sps = int(round(fs / BITRATE))
    nrz = np.repeat(np.array([1.0 if b == "1" else -1.0 for b in packet_bits(header, payload, sync)]), sps)
    t = np.arange(-2 * sps, 2 * sps + 1) / sps
    g = np.exp(-2 * np.pi ** 2 * 0.5 ** 2 * t ** 2 / np.log(2))   # Gaussian, BT = 0.5
    shaped = np.convolve(nrz, g / g.sum(), "same")
    ramp = np.ones(len(shaped)); ramp[:sps] = np.linspace(0, 1, sps); ramp[-sps:] = np.linspace(1, 0, sps)
    phase = np.cumsum(2 * np.pi * DEVIATION_HZ * shaped / fs)
    burst = ramp * np.exp(1j * phase)
    frame = np.zeros(int(fs * FRAME_SECONDS), dtype=np.complex64)
    for k in range(3):
        i = int(k * COPY_SPACING * fs)
        frame[i : i + len(burst)] = burst
    frame *= np.exp(2j * np.pi * offset_hz * np.arange(len(frame)) / fs)
    iq = np.empty(2 * len(frame), dtype=np.int8)
    iq[0::2] = np.round(frame.real * 100)
    iq[1::2] = np.round(frame.imag * 100)
    return iq.tobytes()


def payload_triples(bits, k):
    """The 3x-repeated payload of a packet whose sync starts at bit k."""
    start = k + SYNC_BITS + HEADER_BITS
    if len(bits) < start + 3 * PAYLOAD_BITS:
        return None
    return [bits[start + 3 * n : start + 3 * n + 3] for n in range(PAYLOAD_BITS)]


def find_unknown_sync(bits):
    """Locate the sync of a remote we have no sync for yet.

    Each remote has its own sync word, so try every alignment after the preamble
    and keep the cleanest one whose payload triples are followed by an
    alternating trailer ("1010" or "0101"); clean triples alone can lock on a
    whole triple early or late.
    """
    i = bits.find(PREAMBLE[:8], 0, 70)
    if i < 0:
        return -1, None
    best = (2, -1, None)            # accept at most one bad triple
    for k in range(i, i + 50):
        triples = payload_triples(bits, k)
        if triples is None:
            break
        end = k + SYNC_BITS + HEADER_BITS + 3 * PAYLOAD_BITS
        if bits[end : end + 4] in ("1010", "0101"):
            bad = sum(t not in ("000", "111") for t in triples)
            if bad < best[0]:
                best = (bad, k, triples)
    return best[1], best[2]


def decode_chunk(x, fs, offset_hz, syncs=(SYNC,)):
    """Find and decode remote packets in one chunk of complex samples.

    syncs are the remotes we already know; packets from others are still found
    (see find_unknown_sync) so their controls can be saved.
    """
    sps = int(round(fs / BITRATE))
    x = x - x.mean()

    nfft = 256
    frames = x[: len(x) // nfft * nfft].reshape(-1, nfft)[::8] * np.hanning(nfft)
    psd = (np.abs(np.fft.fftshift(np.fft.fft(frames, axis=1), axes=1)) ** 2).mean(0)
    spectrum = 10 * np.log10(psd.reshape(128, 2).mean(1) / (nfft * 127.0 ** 2) + 1e-12)

    y = x * np.exp(-2j * np.pi * offset_hz * np.arange(len(x)) / fs)
    n = np.arange(-4 * sps, 4 * sps + 1)
    h = np.sinc(1.2e6 / fs * n) * np.hamming(len(n))
    y = np.convolve(y, h / h.sum(), "same")

    w = max(int(fs / 1e5), 1)
    env = np.convolve(np.abs(y), np.ones(w) / w, "same")
    noise = float(np.median(env)) + 1e-9

    pts = 400
    step = len(env) // pts
    envelope = 20 * np.log10(env[: step * pts].reshape(pts, step).max(1) / noise)

    on = env > noise * 6
    edges = np.diff(on.astype(np.int8))
    starts, ends = np.where(edges == 1)[0], np.where(edges == -1)[0]
    if len(starts) and len(ends) and ends[0] < starts[0]:
        ends = ends[1:]
    m = min(len(starts), len(ends))

    packets, bursts, snr = [], 0, 0.0
    margin = 5 * sps
    for s, e in zip(starts[:m], ends[:m]):
        if not 330e-6 < (e - s) / fs < 480e-6 or s < margin or e + margin > len(y):
            continue
        bursts += 1
        snr = max(snr, float(env[s:e].mean() / noise))
        seg = y[s - margin : e + margin]
        fi = np.angle(seg[1:] * np.conj(seg[:-1]))
        fi = np.convolve(fi, np.ones(sps // 2) / (sps // 2), "same")
        core = fi[3 * margin : -3 * margin]
        fi = fi - (np.percentile(core, 10) + np.percentile(core, 90)) / 2
        b = fi > 0
        z = np.where(np.diff(b[2 * margin : -2 * margin]))[0] + 2 * margin
        if len(z) < 20:
            continue
        ph = int(round(np.angle(np.mean(np.exp(2j * np.pi * z / sps))) / (2 * np.pi) * sps)) % sps
        bits = "".join("1" if v else "0" for v in b[(ph + sps // 2) % sps :: sps])
        k = next((i for i in (bits.find(sy, 0, 70) for sy in syncs) if i >= 0), -1)
        if k >= 0:
            triples = payload_triples(bits, k)
            if triples is None or sum(t not in ("000", "111") for t in triples) > 6:
                continue  # collision or bad slice
        else:
            k, triples = find_unknown_sync(bits)
            if triples is None:
                continue
        payload = "".join("1" if t.count("1") >= 2 else "0" for t in triples)
        packets.append((bits[k : k + SYNC_BITS], bits[k + SYNC_BITS : k + SYNC_BITS + HEADER_BITS], payload))

    return {
        "spectrum": spectrum,
        "envelope": envelope,
        "noise_db": 20 * np.log10(noise / 127.0),
        "snr_db": 20 * np.log10(snr) if snr else None,
        "bursts": bursts,
        "packets": packets,
    }


def migrate_labels(labels):
    """Older labels.json was one flat name -> label map, all for the truck."""
    if any("payload" in v for v in labels.values()):
        return {"truck": labels}
    return labels


def is_camera_port(p, camera_arg=None):
    """True for the serial port the ESP32-CAM is on (so the sensor reader leaves it alone)."""
    if camera_arg:
        return p.device == camera_arg
    return p.vid in CAMERA_VIDS


class Proximity:
    """Reads "D <cm>" lines from the HC-SR04 sketch over USB serial.

    blocked() is True while someone is within the stop distance (with hysteresis).
    With no sensor connected the guard simply stays clear.
    """

    def __init__(self, port=None, stop_cm=STOP_CM, camera_arg=None):
        self.port_arg = port
        self.camera_arg = camera_arg
        self.port = None
        self.cm = None
        self.at = 0.0
        self.enabled = True
        self.stop_cm = stop_cm
        self.history = collections.deque(maxlen=RATE_HISTORY)
        self._blocked = False
        self._near = False                         # last reading under stop_cm (for the event log only)
        self._close_at = 0.0                       # time of the last close reading (for SENSOR_HOLD_S)
        self.error = None
        if serial:
            threading.Thread(target=self.run, daemon=True).start()
        else:
            self.error = "pyserial not installed (python3 -m pip install pyserial)"

    def find_port(self):
        if self.port_arg:
            return self.port_arg
        hints = ("usbserial", "usbmodem", "SLAB", "wchusbserial", "ttyUSB", "ttyACM")
        for p in serial.tools.list_ports.comports():
            if any(h in p.device for h in hints) and not is_camera_port(p, self.camera_arg):
                return p.device
        return None

    def run(self):
        while True:
            port = self.find_port()
            if not port:
                if self.port and not self.port.startswith("/dev"):
                    pass                      # readings arrive over the network
                else:
                    self.port = None
                time.sleep(2)
                continue
            try:
                with serial.Serial(port, SENSOR_BAUD, timeout=1) as ser:
                    self.port, self.error = port, None
                    while True:
                        line = ser.readline().decode(errors="replace").strip()
                        if not line.startswith("D "):
                            continue
                        try:
                            cm = int(line[2:])
                        except ValueError:
                            continue
                        now = time.time()
                        self.cm = None if cm < 0 else cm
                        self.at = now
                        self.history.append((now, self.cm))
                        self.note(self.cm)
            except (serial.SerialException, OSError) as ex:
                self.error = f"sensor on {port}: {ex}"
                self.port = None
                time.sleep(2)

    def note(self, cm):
        """Log when a reading first drops under / climbs back over the stop distance."""
        near = cm is not None and cm < self.stop_cm
        if near and not self._near:
            log_event("sensor_near", cm=cm, stop_cm=self.stop_cm)
        elif not near and self._near and (cm is None or cm >= self.stop_cm + RELEASE_CM):
            log_event("sensor_clear", cm=cm)
        else:
            return
        self._near = near

    def report(self, cm, source="wifi"):
        """A reading pushed over the network by the sensor board."""
        now = time.time()
        self.port, self.error = source, None
        self.cm = None if cm < 0 else int(cm)
        self.at = now
        self.history.append((now, self.cm))

    def online(self):
        return self.port is not None and time.time() - self.at < SENSOR_STALE_S

    def blocked(self):
        """Close reading (with RELEASE_CM hysteresis), or one within the last SENSOR_HOLD_S.

        The hold rides out the HC-SR04's 'no echo' gaps on a person; an offline sensor
        still clears the stop at once.
        """
        if not (self.enabled and self.online()):
            self._blocked = False
            return False
        limit = self.stop_cm + RELEASE_CM if self._blocked else self.stop_cm
        if self.cm is not None and self.cm < limit:
            self._close_at = self.at
        self._blocked = self.at - self._close_at < SENSOR_HOLD_S   # latest reading close, or one 0.5 s before it
        return self._blocked

    def state(self):
        now = time.time()
        return {"port": self.port, "online": self.online(), "cm": self.cm, "age": round(now - self.at, 1) if self.at else None,
                "enabled": self.enabled, "stop_cm": self.stop_cm, "blocked": self.blocked(), "error": self.error,
                "history": [[round(t - now, 2), c] for t, c in self.history]}


class Camera:
    """Pulls JPEG frames from the ESP32-CAM (truckcam sketch) over USB serial.

    Bench mode until the Mac can join the TruckCam WiFi. Each "f" sent returns
    "JPG" + 4-byte little-endian length + the JPEG.
    """

    def __init__(self, port=None):
        self.port_arg = port
        self.port = None
        self.jpeg = None
        self.at = 0.0
        self.error = None
        self.times = collections.deque(maxlen=20)
        self.cond = threading.Condition()
        if serial:
            threading.Thread(target=self.run, daemon=True).start()
        else:
            self.error = "pyserial not installed (python3 -m pip install pyserial)"

    def find_port(self):
        for p in serial.tools.list_ports.comports():
            if is_camera_port(p, self.port_arg):
                return p.device
        return None

    def open(self, port):
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = port, 115200, 1
        ser.dtr = ser.rts = False      # the MB wires these to EN/IO0; leave the board running
        ser.open()
        ser.rts = True                 # pulse EN: reboot into a known state (115200, no stale speed)
        time.sleep(0.1)
        ser.rts = False
        time.sleep(2.5)
        ser.write(b"q" + b"b" + struct.pack("<I", CAMERA_BAUD))   # 320x240, then switch speed
        ser.flush()
        time.sleep(0.1)
        ser.baudrate = CAMERA_BAUD
        time.sleep(0.1)
        return ser

    def grab(self, ser):
        ser.reset_input_buffer()
        ser.write(b"f")
        tail, deadline = b"", time.time() + 2
        while tail != b"JPG":
            c = ser.read(1)
            if not c or time.time() > deadline:
                return None
            tail = (tail + c)[-3:]
        n = struct.unpack("<I", ser.read(4))[0]
        if not 0 < n < 300000:
            return None
        data = ser.read(n)
        return data if len(data) == n and data[:2] == b"\xff\xd8" else None

    def run(self):
        while True:
            port = self.find_port()
            if not port:
                self.port = None
                time.sleep(2)
                continue
            try:
                with self.open(port) as ser:
                    self.port, self.error, misses = port, None, 0
                    while misses < 5:          # five bad frames in a row: reboot the board
                        jpeg = self.grab(ser)
                        if not jpeg:
                            misses += 1
                            continue
                        misses = 0
                        with self.cond:
                            self.jpeg, self.at = jpeg, time.time()
                            self.times.append(self.at)
                            self.cond.notify_all()
                    self.error = f"camera on {port} stopped answering; restarting it"
            except (serial.SerialException, OSError) as ex:
                self.error = f"camera on {port}: {ex}"
                self.port = None
                time.sleep(2)

    def wait_frame(self, after, timeout=5):
        """Block until a frame newer than `after` arrives; returns (jpeg, time)."""
        with self.cond:
            self.cond.wait_for(lambda: self.at > after, timeout)
            return self.jpeg, self.at

    def online(self):
        return self.port is not None and time.time() - self.at < CAMERA_STALE_S

    def state(self):
        t = list(self.times)
        fps = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0
        return {"port": self.port, "online": self.online(), "fps": round(fps, 1),
                "kb": round(len(self.jpeg) / 1024, 1) if self.jpeg else None, "error": self.error}


class Detector:
    """Runs YOLO on every camera frame and keeps a copy with boxes drawn.

    Everything the model knows (80 COCO classes) is reported; people and animals count as
    "living" (red boxes), everything else as objects (grey boxes).

    Exposes jpeg / at / wait_frame like Camera, so the dashboard can stream either one.
    Needs `pip install ultralytics`; without it the dashboard shows the plain camera.
    """

    def __init__(self, camera):
        self.camera = camera
        self.jpeg = None
        self.at = 0.0
        self.found = []                            # [{"name", "living", "conf", "box": [x1, y1, x2, y2] as 0-1 fractions}]
        self.ms = None
        self.ready = False
        self.error = None
        self.enabled = True                        # camera safety stop on/off
        self.living_at = 0.0                       # last time a person or animal was in view (any distance)
        self.near_at = 0.0                         # last time one was in range (box tall enough): this holds the truck
        self.min_box_h = None                      # calibrated box height (0-1 of the frame); None = any distance
        try:
            with open(CAMERA_CAL_PATH) as f:
                self.min_box_h = json.load(f).get("min_box_h")
        except (OSError, ValueError):
            pass
        self.cond = threading.Condition()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        try:
            import cv2
            import torch
            from ultralytics import YOLO
            model = YOLO(DETECT_MODEL if os.path.exists(DETECT_MODEL) else os.path.basename(DETECT_MODEL))
            device = "mps" if torch.backends.mps.is_available() else "cpu"
        except Exception as ex:                    # missing package or model: keep the plain camera
            self.error = f"person detection off: {ex}"
            return
        self.ready = True
        last = 0.0
        while True:
            jpeg, at = self.camera.wait_frame(last)
            if not jpeg or at <= last:
                continue
            last = at
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            t = time.time()
            r = model.predict(img, imgsz=DETECT_IMGSZ, conf=DETECT_CONF, device=device, verbose=False)[0]
            ms = (time.time() - t) * 1e3
            h, w = img.shape[:2]
            found = []
            for (x1, y1, x2, y2), conf, cls in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(), r.boxes.cls.tolist()):
                name = model.names[int(cls)]
                living = name in LIVING
                found.append({"name": name, "living": living, "conf": round(conf, 2),
                              "box": [round(x1 / w, 3), round(y1 / h, 3), round(x2 / w, 3), round(y2 / h, 3)]})
                color = (77, 72, 229) if living else (170, 170, 170)    # BGR: red for living, grey for objects
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2 if living else 1)
                cv2.putText(img, f"{name} {conf:.0%}", (int(x1) + 3, max(12, int(y1) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue
            with self.cond:
                self.jpeg, self.at, self.found, self.ms = buf.tobytes(), at, found, ms
                for f in found:
                    f["near"] = f["living"] and self.in_range(f)
                if any(f["living"] for f in found):
                    self.living_at = time.time()
                near = [f for f in found if f["near"]]
                if near:
                    if time.time() - self.near_at >= CAMERA_HOLD_S:
                        log_event("camera_person", frame_at=round(at, 3), infer_ms=round(ms),
                                  conf=max(f["conf"] for f in near), box_h=max(self.box_h(f) for f in near))
                    self.near_at = time.time()
                self.cond.notify_all()

    def wait_frame(self, after, timeout=5):
        with self.cond:
            self.cond.wait_for(lambda: self.at > after, timeout)
            return self.jpeg, self.at

    def blocked(self):
        """True while a person or animal was seen in the last CAMERA_HOLD_S.

        Like the distance sensor, a camera that stops sending frames clears the stop.
        """
        return self.enabled and self.ready and time.time() - self.near_at < CAMERA_HOLD_S

    @staticmethod
    def box_h(f):
        return round(f["box"][3] - f["box"][1], 3)

    def in_range(self, f):
        return self.min_box_h is None or self.box_h(f) >= self.min_box_h * CAMERA_CAL_SLACK

    def calibrate(self, clear=False):
        """Set the range from the tallest person in view now (standing on the stop-distance mark)."""
        if clear:
            self.min_box_h = None
        else:
            if time.time() - self.at > CAMERA_STALE_S:
                return "No fresh camera frame."
            people = [f for f in self.found if f["name"] == "person"]
            if not people:
                return "No person in view: stand on the 6 ft mark, facing the camera, and try again."
            self.min_box_h = max(self.box_h(f) for f in people)
        try:
            with open(CAMERA_CAL_PATH, "w") as f:
                json.dump({"min_box_h": self.min_box_h, "at": time.time()}, f)
        except OSError:
            pass
        log_event("camera_calibrated", min_box_h=self.min_box_h)
        return None

    def state(self):
        fresh = time.time() - self.at < CAMERA_STALE_S
        found = self.found if fresh else []
        living = [f for f in found if f["living"]]
        return {"ready": self.ready, "error": self.error, "fresh": fresh,
                "living": len(living), "persons": sum(f["name"] == "person" for f in found),
                "best_conf": max((f["conf"] for f in living), default=None),
                "found": found, "ms": round(self.ms, 1) if self.ms is not None else None,
                "enabled": self.enabled, "blocked": self.blocked(), "hold_s": CAMERA_HOLD_S,
                "min_box_h": self.min_box_h, "near": sum(bool(f.get("near")) for f in found)}


TRIAL_S = 8.0            # each trial records this long after Start
TRIAL_ORDER = os.path.join(HERE, "data", "trial_order.csv")    # made by make_trials.py
TRIALS_CSV = os.path.join(HERE, "data", "trials.csv")
TRIAL_COLS = ["trial", "date", "time", "distance_ft", "lighting", "target",
              "camera_detected", "camera_first_s", "camera_conf", "camera_box_h", "camera_seen_any_distance",
              "ultrasonic_detected", "ultrasonic_first_s", "ultrasonic_cm", "ultrasonic_min_cm",
              "blocked_at_start", "truck_driving", "truck_stopped", "stopped_by", "stop_at_s",
              "mac_reaction_ms", "time_to_stop_s", "stop_cm", "safety_stop_on", "camera_stop_on", "notes"]


class Trials:
    """Records one TRIAL_PLAN.md trial at a time into data/trials.csv.

    Start a trial, bring the target in during the TRIAL_S window, and the row is saved
    automatically: what the camera and the ultrasonic sensor saw, whether the transmitter
    switched to idle, and when (seconds after Start). Detection is recorded even when that
    sensor's stop is switched off. time_to_stop_s (from slow-motion video) and notes are
    filled in afterwards. Re-running a trial number replaces its row.
    """

    def __init__(self, radio):
        self.radio = radio
        self.running = None                        # the trial dict while recording
        self.t0 = 0.0
        self.last = None                           # last saved row
        self.error = None
        self.lock = threading.Lock()

    def order(self):
        try:
            import csv
            with open(TRIAL_ORDER, newline="") as f:
                return {int(r["trial"]): r for r in csv.DictReader(f)}
        except (OSError, ValueError, KeyError):
            return {}

    def rows(self):
        try:
            import csv
            with open(TRIALS_CSV, newline="") as f:
                return {int(r["trial"]): r for r in csv.DictReader(f)}
        except (OSError, ValueError, KeyError):
            return {}

    def write(self, rows):
        import csv
        os.makedirs(os.path.dirname(TRIALS_CSV), exist_ok=True)
        tmp = TRIALS_CSV + ".tmp"
        with open(tmp, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=TRIAL_COLS, extrasaction="ignore")
            w.writeheader()
            for k in sorted(rows):
                w.writerow(rows[k])
        os.replace(tmp, TRIALS_CSV)

    def start(self, trial, notes=""):
        with self.lock:
            if self.running:
                return "A trial is already recording."
            spec = self.order().get(trial)
            if not spec:
                return f"Trial {trial} is not in data/trial_order.csv (run make_trials.py)."
            self.running = dict(spec, notes=notes)
            self.t0 = time.time()
        threading.Thread(target=self.record, daemon=True).start()
        log_event("trial_start", trial=trial)
        return None

    def record(self):
        r, prox, det, t0 = self.radio, self.radio.proximity, self.radio.detector, self.t0
        stop_cm = prox.stop_cm
        readings, cam_first, cam_conf, driving, held, drive_first = [], None, 0.0, False, False, None
        last_at, near0, living0 = prox.at, det.near_at if det else 0.0, det.living_at if det else 0.0
        seen_any, box_h = False, 0.0
        blocked0 = r.blocked()
        while time.time() - t0 < TRIAL_S:
            if prox.at != last_at:
                last_at = prox.at
                readings.append((last_at - t0, prox.cm))
            if det and det.living_at > max(t0, living0):
                seen_any = True
            if det and det.near_at > max(t0, near0):
                if cam_first is None:
                    cam_first = det.near_at - t0
                near = [f for f in det.found if f.get("near")]
                cam_conf = max([cam_conf] + [f["conf"] for f in near])
                box_h = max([box_h] + [det.box_h(f) for f in near])
            tx = r.tx_label
            moving = bool(tx and tx[1] != "idle")
            if moving and drive_first is None:
                drive_first = time.time() - t0
            driving = driving or moving
            held = held or (moving and r.guarded(tx) != tx)   # covers a target already there at Start
            time.sleep(0.03)
        near = [(t, cm) for t, cm in readings if cm is not None and cm < stop_cm]
        idle = [e for e in EVENTS if e["event"] == "tx_idle" and t0 <= e["t"] <= t0 + TRIAL_S]
        firsts = [x for x in (cam_first, near[0][0] if near else None) if x is not None]
        stop_at = idle[0]["t"] - t0 if idle else None
        by = []
        if idle:
            by = [k for k in ("sensor", "camera") if idle[0].get(k)]
        cms = sorted(cm for _, cm in near)
        row = dict(self.running,
                   date=time.strftime("%Y-%m-%d", time.localtime(t0)), time=time.strftime("%H:%M:%S", time.localtime(t0)),
                   camera_detected="yes" if cam_first is not None else "no",
                   camera_first_s=None if cam_first is None else round(cam_first, 2),
                   camera_conf=round(cam_conf, 2) if cam_first is not None else None,
                   camera_box_h=round(box_h, 3) if cam_first is not None else None,
                   camera_seen_any_distance="yes" if seen_any or cam_first is not None else "no",
                   ultrasonic_detected="yes" if near else "no",
                   ultrasonic_first_s=round(near[0][0], 2) if near else None,
                   ultrasonic_cm=cms[len(cms) // 2] if cms else None,
                   ultrasonic_min_cm=cms[0] if cms else None,
                   blocked_at_start="yes" if blocked0 else "no",
                   truck_driving="yes" if driving else "no",
                   truck_stopped="yes" if idle or held else "no",
                   stopped_by="+".join(by) or None,
                   stop_at_s=None if stop_at is None else round(stop_at, 2),
                   mac_reaction_ms=self.reaction(stop_at, firsts, drive_first),
                   time_to_stop_s=None, stop_cm=stop_cm,
                   safety_stop_on="yes" if prox.enabled else "no",
                   camera_stop_on="yes" if det and det.enabled else "no")
        try:
            rows = self.rows()
            rows[int(row["trial"])] = row
            self.write(rows)
            self.error = None
        except OSError as ex:
            self.error = f"could not save data/trials.csv: {ex}"
        log_event("trial_saved", trial=int(row["trial"]), truck_stopped=row["truck_stopped"])
        with self.lock:
            self.last, self.running = row, None

    @staticmethod
    def reaction(stop_at, firsts, drive_first):
        """ms from 'target seen while driving' to the switch to idle; None if it can't be measured."""
        if stop_at is None or not firsts or drive_first is None:
            return None
        start = max(min(firsts), drive_first)     # target already there before ↑ was pressed: count from ↑
        return round((stop_at - start) * 1e3) if stop_at >= start else None

    def annotate(self, trial, fields):
        """Fill in notes / time_to_stop_s on a saved row."""
        with self.lock:
            rows = self.rows()
            if trial not in rows:
                return f"Trial {trial} has not been recorded yet."
            for k in ("notes", "time_to_stop_s"):
                if k in fields:
                    rows[trial][k] = fields[k]
            self.write(rows)
            if self.last and int(self.last["trial"]) == trial:
                self.last = rows[trial]
        return None

    def state(self):
        order, rows = self.order(), self.rows()
        nxt = next((n for n in sorted(order) if n not in rows), None)
        run = self.running
        return {"total": len(order), "done": len(rows), "next": order.get(nxt) if nxt else None,
                "order": order, "recorded": sorted(rows),
                "running": run, "remaining_s": round(max(0.0, TRIAL_S - (time.time() - self.t0)), 1) if run else None,
                "window_s": TRIAL_S, "last": self.last, "error": self.error}


class Radio:
    def __init__(self, args):
        self.args = args
        self.proximity = Proximity(args.sensor, args.stop_cm, args.camera)
        self.camera = Camera(args.camera)
        self.detector = None if args.no_detect else Detector(self.camera)
        self.guard_stops = 0                       # how often the safety stop has kicked in
        self.lock = threading.Lock()
        self.channel = CHANNELS[0]
        self.proc = None
        self.error = None
        self.latest = None
        self.restart = False
        self.rate_history = collections.deque(maxlen=RATE_HISTORY)
        self.recent = collections.deque()          # (time, sync, header, payload)
        self.seen = {}                             # payload -> [count, first, last, header, sync]
        self.last = {}
        self.frame = None
        self.tx_label = None                       # (vehicle, name) while transmitting
        self.labels = {}                           # vehicle -> name -> label
        if os.path.exists(LABELS_PATH):
            with open(LABELS_PATH) as f:
                self.labels = migrate_labels(json.load(f))

    # --- sample source -----------------------------------------------------
    @property
    def fs(self):
        return self.args.fs

    @property
    def offset(self):
        return self.args.offset if self.args.file else -1e6

    def reader(self):
        nbytes = int(self.fs * CHUNK_SECONDS) * 2
        while True:
            self.restart = False
            try:
                if self.tx_label:
                    self.transmit(self.tx_label)
                elif self.args.file:
                    self.read_file(nbytes)
                else:
                    self.read_hackrf(nbytes)
            except Exception as ex:  # keep the dashboard up whatever the radio does
                if not self.restart:   # a deliberate RX/TX switch is not an error
                    self.error = str(ex)
            if not self.restart:
                time.sleep(2)

    def read_file(self, nbytes):
        with open(self.args.file, "rb") as f:
            while not self.restart:
                buf = f.read(nbytes)
                if len(buf) < nbytes:
                    f.seek(0)
                    continue
                self.error = None
                self.frame = buf
                time.sleep(CHUNK_SECONDS)

    def read_hackrf(self, nbytes):
        cmd = ["hackrf_transfer", "-r", "-", "-f", str(int((self.channel + 1) * 1e6)),
               "-s", str(int(self.fs)), "-l", "24", "-g", "26", "-a", "0"]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            while not self.restart:
                buf = self.proc.stdout.read(nbytes)
                if len(buf) < nbytes:
                    err = self.proc.stderr.read().decode(errors="replace").strip().splitlines()
                    msg = next((l for l in err if "fail" in l.lower() or "not found" in l.lower()), None)
                    raise RuntimeError(msg or "HackRF stream stopped")
                self.error = None
                self.frame = buf
        finally:
            self.proc.kill()
            self.proc.wait()

    def label(self, key):
        vehicle, name = key
        return self.labels.get(vehicle, {}).get(name)

    def blocked(self):
        """Someone is too close (distance sensor) or in view (camera)."""
        return self.proximity.blocked() or bool(self.detector and self.detector.blocked())

    def guarded(self, key):
        """The control actually sent: idle (or None = stop) while someone is too close or in view."""
        if key is None or key[1] == "idle" or not self.blocked():
            return key
        idle = (key[0], "idle")
        return idle if self.label(idle) else None

    def transmit(self, key):
        """Stream frames to one hackrf_transfer process; swap the packet without restarting.

        Transmits on the channel the control was saved on; a control on another
        channel restarts the transmitter there.
        """
        first = self.label(key)
        if not first:
            self.tx_label = None
            return
        mhz = first.get("channel", self.channel)
        cmd = ["hackrf_transfer", "-t", "-", "-f", str(int((mhz + 1) * 1e6)),
               "-s", str(int(self.fs)), "-x", str(TX_GAIN), "-a", "0"]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        frames, cur, retune, guarding = {}, None, False, False
        try:
            while self.tx_label and self.proc.poll() is None:
                want = self.guarded(self.tx_label)
                if want != self.tx_label and not guarding:
                    self.guard_stops += 1
                    log_event("tx_idle", control=self.tx_label[1],
                              sensor=self.proximity.blocked(), camera=bool(self.detector and self.detector.blocked()))
                elif want == self.tx_label and guarding:
                    log_event("tx_resume" if self.tx_label[1] != "idle" else "tx_released", control=self.tx_label[1])
                guarding = want != self.tx_label
                if want is None:               # too close and no idle packet saved: stop
                    break
                if want != cur:
                    cur = want
                    label = self.label(cur)
                    if not label:
                        break
                    if label.get("channel", self.channel) != mhz:
                        retune = True
                        break
                    if cur not in frames:
                        frames[cur] = synth_frame(label["header"], label["payload"], self.fs, self.offset,
                                                  label.get("sync", SYNC))
                self.proc.stdin.write(frames[cur])   # blocks at the radio's pace
            if self.proc.poll() is not None and self.tx_label:
                err = self.proc.stderr.read().decode(errors="replace").strip().splitlines()
                self.tx_label = None
                raise RuntimeError(next((l for l in err if "fail" in l.lower()), None) or "transmit stopped")
            self.error = None
        except BrokenPipeError:
            pass
        finally:
            if not retune:
                self.tx_label = None
            self.proc.kill()
            self.proc.wait()
            self.restart = True

    def drive(self, vehicle, name):
        """Start transmitting a saved control, switch control, or stop with name=None."""
        key = None if name is None else (vehicle, name)
        if key is not None and not self.label(key):
            return False
        was = self.tx_label
        self.tx_label = key
        if was and key:
            return True            # transmitter keeps running with the new frame
        self.restart = True
        if self.proc:
            self.proc.kill()
        return True

    # --- processing --------------------------------------------------------
    def worker(self):
        while True:
            buf, self.frame = self.frame, None
            if buf is None:
                time.sleep(0.01)
                continue
            d = np.frombuffer(buf, dtype=np.int8).astype(np.float32)
            syncs = [SYNC] + sorted({l.get("sync", SYNC) for v in self.labels.values() for l in v.values()} - {SYNC})
            r = decode_chunk(d[0::2] + 1j * d[1::2], self.fs, self.offset, syncs)
            now = time.time()
            with self.lock:
                self.last = r
                self.rate_history.append((now, r["bursts"] / CHUNK_SECONDS))
                for sync, header, payload in r["packets"]:
                    self.recent.append((now, sync, header, payload))
                    s = self.seen.setdefault(payload, [0, now, now, header, sync])
                    s[0] += 1
                    s[2] = now
                while self.recent and self.recent[0][0] < now - 1.0:
                    self.recent.popleft()

    def set_channel(self, mhz):
        with self.lock:
            self.channel = mhz
            self.recent.clear()
            self.rate_history.clear()
            self.seen.clear()
        self.restart = True
        if self.proc:
            self.proc.kill()

    def current(self):
        if not self.recent:
            return None
        payload, n = collections.Counter(p for _, _, _, p in self.recent).most_common(1)[0]
        sync, header = next((sy, h) for _, sy, h, p in self.recent if p == payload)
        return {"sync": sync, "header": header, "payload": payload, "count": n,
                "vehicle": self.vehicle_for(sync)}

    def vehicle_for(self, sync):
        """Which vehicle already has controls saved from the remote with this sync."""
        for vehicle, labels in self.labels.items():
            if any(l.get("sync", SYNC) == sync for l in labels.values()):
                return vehicle
        return None

    def save_label(self, vehicle, name):
        """Returns None on success, else why the packet was not saved."""
        with self.lock:
            cur = self.current()
            if not cur:
                return "Nothing to save: no packet decoded in the last second."
            if cur["vehicle"] not in (None, vehicle):
                return f"That packet is from the {cur['vehicle']}'s remote, not the {vehicle}'s."
            self.labels.setdefault(vehicle, {})[name] = {"sync": cur["sync"], "header": cur["header"], "payload": cur["payload"],
                                 "channel": self.channel, "saved": time.time()}
            self.write_labels()
        return None

    def delete_label(self, vehicle, name):
        with self.lock:
            self.labels.get(vehicle, {}).pop(name, None)
            self.write_labels()

    def write_labels(self):
        with open(LABELS_PATH, "w") as f:
            json.dump(self.labels, f, indent=2)

    def state(self):
        try:  # pick up edits made to labels.json by hand
            mtime = os.path.getmtime(LABELS_PATH)
            if mtime != getattr(self, "_labels_mtime", None):
                with open(LABELS_PATH) as f:
                    self.labels = migrate_labels(json.load(f))
                self._labels_mtime = mtime
        except (OSError, ValueError):
            pass
        with self.lock:
            now = time.time()
            r = self.last
            rates = [v for t, v in self.rate_history if t > now - 1.0]
            center = (2410 if self.args.file else self.channel + 1)
            return {
                "started": STARTED,                # lets the page notice a server restart
                "error": self.error,
                "transmitting": dict(zip(("vehicle", "name"), self.tx_label)) if self.tx_label else None,
                "proximity": self.proximity.state(),
                "camera": self.camera.state(),
                "detect": self.detector.state() if self.detector else {"ready": False, "error": "off (--no-detect)"},
                "guard_stops": self.guard_stops,
                "vehicles": VEHICLES,
                "source": "file" if self.args.file else "hackrf",
                "channel": self.channel,
                "channels": CHANNELS,
                "center_mhz": center,
                "span_mhz": self.fs / 1e6,
                "packets_per_s": sum(rates) / len(rates) if rates else 0,
                "snr_db": r.get("snr_db"),
                "noise_db": r.get("noise_db"),
                "spectrum": [round(float(v), 1) for v in r.get("spectrum", [])],
                "envelope": [round(float(v), 1) for v in r.get("envelope", [])],
                "window_ms": CHUNK_SECONDS * 1e3,
                "rate_history": [[round(t - now, 2), v] for t, v in self.rate_history],
                "current": self.current(),
                "seen": sorted(
                    ({"payload": p, "header": s[3], "sync": s[4], "vehicle": self.vehicle_for(s[4]),
                      "count": s[0], "age": round(now - s[2], 1)}
                     for p, s in self.seen.items() if s[0] >= 3),
                    key=lambda d: -d["count"])[:8],
                "labels": self.labels,
            }


class Handler(BaseHTTPRequestHandler):
    radio = None

    def log_message(self, *a):
        pass

    def send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/api/state":
            return self.send_json(self.radio.state())
        if path == "/api/trials":
            return self.send_json(self.radio.trials.state())
        if path == "/api/events":                # stop-time test timeline
            return self.send_json({"events": list(EVENTS), "stops": event_summary()})
        if path == "/api/camera.jpg":            # latest frame (for person detection)
            jpeg = self.radio.camera.jpeg
            if not jpeg:
                return self.send_error(503, "no camera frame yet")
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            return self.wfile.write(jpeg)
        if path == "/api/camera.mjpg":           # live view; boxes drawn when detection is running
            det = self.radio.detector
            raw = "raw=1" in self.path or not (det and det.ready)
            return self.stream_camera(self.radio.camera if raw else det)
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return self.wfile.write(body)
        self.send_error(404)

    def stream_camera(self, cam):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = 0.0
        try:
            while True:
                jpeg, at = cam.wait_frame(last)
                if at <= last:
                    continue
                last = at
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass                                  # page closed

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            data = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self.send_json({"ok": False}, 400)
        vehicle = data.get("vehicle")
        if self.path in ("/api/label", "/api/label/delete", "/api/drive") and vehicle not in VEHICLES:
            return self.send_json({"ok": False}, 400)
        if self.path == "/api/label":
            name = str(data.get("name", "")).strip()[:40]
            err = self.radio.save_label(vehicle, name) if name else "Type a label first."
            return self.send_json({"ok": not err, "error": err}, 409 if err else 200)
        if self.path == "/api/label/delete":
            self.radio.delete_label(vehicle, str(data.get("name", "")))
            return self.send_json({"ok": True})
        if self.path == "/api/drive":
            name = data.get("name")
            ok = self.radio.drive(vehicle, None if name in (None, "") else str(name))
            return self.send_json({"ok": ok}, 200 if ok else 404)
        if self.path == "/api/proximity":        # pushed by the ESP32 over WiFi
            try:
                self.radio.proximity.report(int(data["cm"]), data.get("source", "wifi"))
            except (KeyError, TypeError, ValueError):
                return self.send_json({"ok": False}, 400)
            return self.send_json({"ok": True})
        if self.path == "/api/guard":
            prox = self.radio.proximity
            if "enabled" in data:
                prox.enabled = bool(data["enabled"])
            if "stop_cm" in data:
                try:
                    prox.stop_cm = max(10, min(400, int(data["stop_cm"])))
                except (TypeError, ValueError):
                    return self.send_json({"ok": False}, 400)
            if "camera" in data and self.radio.detector:
                self.radio.detector.enabled = bool(data["camera"])
            return self.send_json({"ok": True})
        if self.path == "/api/camera/calibrate":
            if not self.radio.detector:
                return self.send_json({"ok": False, "error": "detection is off"}, 409)
            err = self.radio.detector.calibrate(clear=bool(data.get("clear")))
            return self.send_json({"ok": not err, "error": err}, 409 if err else 200)
        if self.path in ("/api/trial/start", "/api/trial/note"):
            try:
                trial = int(data["trial"])
            except (KeyError, TypeError, ValueError):
                return self.send_json({"ok": False, "error": "trial number missing"}, 400)
            if self.path == "/api/trial/start":
                err = self.radio.trials.start(trial, str(data.get("notes", ""))[:200])
            else:
                fields = {k: str(data[k])[:200] for k in ("notes", "time_to_stop_s") if k in data}
                err = self.radio.trials.annotate(trial, fields)
            return self.send_json({"ok": not err, "error": err}, 409 if err else 200)
        if self.path == "/api/channel":
            mhz = data.get("mhz")
            if mhz not in CHANNELS:
                return self.send_json({"ok": False}, 400)
            self.radio.set_channel(mhz)
            return self.send_json({"ok": True})
        self.send_error(404)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", help="replay an int8 IQ capture instead of using the HackRF")
    ap.add_argument("--fs", type=float, default=8e6, help="sample rate (default 8e6)")
    ap.add_argument("--offset", type=float, default=0, help="with --file: channel offset from centre, Hz")
    ap.add_argument("--sensor", help="serial port of the HC-SR04 board (default: first USB serial port found)")
    ap.add_argument("--camera", help="serial port of the ESP32-CAM (default: the CH340 USB port)")
    ap.add_argument("--no-detect", action="store_true", help="don't run YOLO person detection on the camera")
    ap.add_argument("--stop-cm", type=int, default=STOP_CM, help=f"safety-stop distance in cm (default {STOP_CM} = 6 ft)")
    args = ap.parse_args()

    radio = Radio(args)
    radio.trials = Trials(radio)
    Handler.radio = radio
    threading.Thread(target=radio.reader, daemon=True).start()
    threading.Thread(target=radio.worker, daemon=True).start()
    print(f"Truck remote dashboard: http://localhost:{PORT}")
    # 0.0.0.0 so the sensor board on the WiFi can post readings; the page itself is still local-network only.
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
