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
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LABELS_PATH = os.path.join(HERE, "labels.json")
PORT = 8642

# Truck: 2415/2435/2470. Loader: 2418/2438/2458/2473 (re-pairing a remote can move it).
CHANNELS = [2415, 2418, 2435, 2438, 2458, 2470, 2473]
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
TX_FRAME_PATH = os.path.join(HERE, "tx_frame.iq")


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
        frames, cur, retune = {}, None, False
        try:
            while self.tx_label and self.proc.poll() is None:
                if self.tx_label != cur:
                    cur = self.tx_label
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
                "error": self.error,
                "transmitting": dict(zip(("vehicle", "name"), self.tx_label)) if self.tx_label else None,
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
        if self.path == "/api/state":
            return self.send_json(self.radio.state())
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
