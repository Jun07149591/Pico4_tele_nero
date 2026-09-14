from collections import deque
import json
from pathlib import Path
import socket
import threading
import time


class TelemetryReceiver:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()
        self.history = deque(maxlen=500)
        self.stop = threading.Event()
        self.error = None
        self.rejected_packets = 0
        self.sock = None
        self.worker = None

    def __enter__(self):
        if len(str(self.path).encode()) > 100:
            raise ValueError("telemetry socket path is too long")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            self.sock.bind(str(self.path))
        except OSError:
            self.sock.close()
            raise RuntimeError(f"socket already exists/in use: {self.path}; do not run two collectors")
        self.path.chmod(0o600)
        self.inode = self.path.stat().st_ino
        self.sock.settimeout(.1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)

        def run():
            while not self.stop.is_set():
                try:
                    raw = self.sock.recv(65536)
                    packet = json.loads(raw)
                    if packet["schema_version"] != 1 or not isinstance(packet["arms"], dict):
                        raise ValueError("unsupported telemetry")
                    if not 0 <= time.monotonic() - packet["monotonic"] <= .5:
                        raise ValueError("stale/future telemetry")
                    with self.lock:
                        previous = self.history[-1] if self.history else None
                        if previous and packet["session_id"] == previous["session_id"]:
                            if packet["sequence"] <= previous["sequence"] or packet["monotonic"] <= previous["monotonic"]:
                                raise ValueError("non-increasing telemetry")
                        self.history.append(packet)
                except socket.timeout:
                    continue
                except (KeyError, ValueError, TypeError):
                    self.rejected_packets += 1
                except OSError as exc:
                    self.error = str(exc)
                    return

        self.worker = threading.Thread(target=run, name="data-telemetry", daemon=True)
        self.worker.start()
        return self

    def latest_before(self, timestamp):
        with self.lock:
            return next((p for p in reversed(self.history) if p["monotonic"] <= timestamp), None)

    def latest(self):
        with self.lock:
            return self.history[-1] if self.history else None

    def snapshot(self):
        with self.lock:
            return list(self.history)

    def between(self, start, end):
        with self.lock:
            return [p for p in self.history if start < p["monotonic"] <= end]

    def __exit__(self, *_args):
        self.stop.set()
        self.worker.join(timeout=1.)
        self.sock.close()
        if self.path.exists() and self.path.stat().st_ino == self.inode:
            self.path.unlink()
