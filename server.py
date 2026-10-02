#!/usr/bin/env python3
"""Live dashboard for the toy truck's 2.4 GHz remote.

Streams IQ from a HackRF, finds the remote's packets, decodes them and serves
a web page at http://localhost:8642.

    python3 server.py                     # live from the HackRF
    python3 server.py --file ../cap_2410.iq --fs 20e6 --offset 5e6   # replay a capture
"""
import argparse
import collections
import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LABELS_PATH = os.path.join(HERE, "labels.json")
PORT = 8642

CHANNELS = [2415, 2435, 2470]
BITRATE = 1e6
SYNC = "0001111110110"   # follows the 1010... preamble
HEADER_BITS = 19         # raw bits between sync and the 3x-repeated payload
PAYLOAD_BITS = 112
CHUNK_SECONDS = 0.1
RATE_HISTORY = 150       # chunks kept for the packet-rate chart
PREAMBLE = "10" * 16
DEVIATION_HZ = 340e3     # measured on the real remote
FRAME_SECONDS = 0.02     # one packet group every 20 ms
COPY_SPACING = 0.0022    # three copies 2.2 ms apart
TX_GAIN = 20
TX_FRAME_PATH = os.path.join(HERE, "tx_frame.iq")


def packet_bits(header, payload):
    return PREAMBLE + SYNC + header + "".join(b * 3 for b in payload) + "1010"


def synth_frame(header, payload, fs, offset_hz):
    """One 20 ms frame of GFSK (three packet copies) as int8 IQ, shifted by offset_hz."""
    sps = int(round(fs / BITRATE))
    nrz = np.repeat(np.array([1.0 if b == "1" else -1.0 for b in packet_bits(header, payload)]), sps)
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


def decode_chunk(x, fs, offset_hz):
    """Find and decode remote packets in one chunk of complex samples."""
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
        i = bits.find(SYNC, 0, 70)
        if i < 0:
            continue
        body = bits[i + len(SYNC) :]
        if len(body) < HEADER_BITS + 3 * PAYLOAD_BITS:
            continue
        triples = [body[HEADER_BITS + 3 * k : HEADER_BITS + 3 * k + 3] for k in range(PAYLOAD_BITS)]
        if sum(t not in ("000", "111") for t in triples) > 6:
            continue  # collision or bad slice
        payload = "".join("1" if t.count("1") >= 2 else "0" for t in triples)
        packets.append((body[:HEADER_BITS], payload))

    return {
        "spectrum": spectrum,
        "envelope": envelope,
        "noise_db": 20 * np.log10(noise / 127.0),
        "snr_db": 20 * np.log10(snr) if snr else None,
        "bursts": bursts,
        "packets": packets,
    }


class Radio:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.channel = CHANNELS[0]
        self.proc = None
        self.error = None
        self.latest = None
        self.restart = False
        self.rate_history = collections.deque(maxlen=RATE_HISTORY)
        self.recent = collections.deque()          # (time, header, payload)
        self.seen = {}                             # payload -> [count, first, last, header]
        self.last = {}
        self.frame = None
        self.tx_label = None
        self.labels = {}
        if os.path.exists(LABELS_PATH):
            with open(LABELS_PATH) as f:
                self.labels = json.load(f)

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

    def transmit(self, name):
        """Stream frames to one hackrf_transfer process; swap the packet without restarting."""
        cmd = ["hackrf_transfer", "-t", "-", "-f", str(int((self.channel + 1) * 1e6)),
               "-s", str(int(self.fs)), "-x", str(TX_GAIN), "-a", "0"]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        frames, cur = {}, None
        try:
            while self.tx_label and self.proc.poll() is None:
                if self.tx_label != cur:
                    cur = self.tx_label
                    label = self.labels.get(cur)
                    if not label:
                        break
                    if cur not in frames:
                        frames[cur] = synth_frame(label["header"], label["payload"], self.fs, self.offset)
                self.proc.stdin.write(frames[cur])   # blocks at the radio's pace
            if self.proc.poll() is not None and self.tx_label:
                err = self.proc.stderr.read().decode(errors="replace").strip().splitlines()
                self.tx_label = None
                raise RuntimeError(next((l for l in err if "fail" in l.lower()), None) or "transmit stopped")
            self.error = None
        except BrokenPipeError:
            pass
        finally:
            self.tx_label = None
            self.proc.kill()
            self.proc.wait()
            self.restart = True

    def drive(self, name):
        """Start transmitting a saved control, switch control, or stop with name=None."""
        if name is not None and name not in self.labels:
            return False
        was = self.tx_label
        self.tx_label = name
        if was and name:
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
            r = decode_chunk(d[0::2] + 1j * d[1::2], self.fs, self.offset)
            now = time.time()
            with self.lock:
                self.last = r
                self.rate_history.append((now, r["bursts"] / CHUNK_SECONDS))
                for header, payload in r["packets"]:
                    self.recent.append((now, header, payload))
                    s = self.seen.setdefault(payload, [0, now, now, header])
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
        payload, n = collections.Counter(p for _, _, p in self.recent).most_common(1)[0]
        header = next(h for _, h, p in self.recent if p == payload)
        return {"header": header, "payload": payload, "count": n}

    def save_label(self, name):
        with self.lock:
            cur = self.current()
            if not cur:
                return False
            self.labels[name] = {"header": cur["header"], "payload": cur["payload"],
                                 "channel": self.channel, "saved": time.time()}
            self.write_labels()
        return True

    def delete_label(self, name):
        with self.lock:
            self.labels.pop(name, None)
            self.write_labels()

    def write_labels(self):
        with open(LABELS_PATH, "w") as f:
            json.dump(self.labels, f, indent=2)

    def state(self):
        try:  # pick up edits made to labels.json by hand
            mtime = os.path.getmtime(LABELS_PATH)
            if mtime != getattr(self, "_labels_mtime", None):
                with open(LABELS_PATH) as f:
                    self.labels = json.load(f)
                self._labels_mtime = mtime
        except (OSError, ValueError):
            pass
        with self.lock:
            now = time.time()
            r = self.last
            rates = [v for t, v in self.rate_history if t > now - 1.0]
            center = (2410 if self.args.file else self.channel + 1)
            return {
                "error": self.error,
                "transmitting": self.tx_label,
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
                    ({"payload": p, "header": s[3], "count": s[0], "age": round(now - s[2], 1)}
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
        if self.path == "/api/state":
            return self.send_json(self.radio.state())
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return self.wfile.write(body)
        self.send_error(404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            data = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self.send_json({"ok": False}, 400)
        if self.path == "/api/label":
            name = str(data.get("name", "")).strip()[:40]
            ok = bool(name) and self.radio.save_label(name)
            return self.send_json({"ok": ok}, 200 if ok else 409)
        if self.path == "/api/label/delete":
            self.radio.delete_label(str(data.get("name", "")))
            return self.send_json({"ok": True})
        if self.path == "/api/drive":
            name = data.get("name")
            ok = self.radio.drive(None if name in (None, "") else str(name))
            return self.send_json({"ok": ok}, 200 if ok else 404)
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
    args = ap.parse_args()

    radio = Radio(args)
    Handler.radio = radio
    threading.Thread(target=radio.reader, daemon=True).start()
    threading.Thread(target=radio.worker, daemon=True).start()
    print(f"Truck remote dashboard: http://localhost:{PORT}")
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
