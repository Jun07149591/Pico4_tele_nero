from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import re
import threading
import time
from urllib.parse import parse_qs, urlparse

import cv2
import h5py
import numpy as np

from .export import export_dataset, select_episodes
from .quality import episode_path, list_episodes, summary, validate_episode

STATIC = Path(__file__).with_name("static")


def create_server(controller, port=8765):
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="dataset-export")
    job_lock = threading.Lock()
    export_job = {"state": "idle", "result": None, "error": None, "progress": None}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *_args):
            pass

        def send(self, value, content_type="application/json", status=200):
            payload = json.dumps(value, ensure_ascii=False, allow_nan=False).encode() if content_type == "application/json" else value
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' blob:; script-src 'self'; style-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(payload)

        def check_host(self):
            return self.headers.get("Host") in (f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}")

        def do_GET(self):
            if not self.check_host():
                self.send({"error": "invalid host"}, status=403)
                return
            try:
                parsed = urlparse(self.path)
                params = {key: values[0] for key, values in parse_qs(parsed.query).items()}
                if parsed.path == "/api/status":
                    with job_lock:
                        job = dict(export_job)
                    self.send({**controller.status(), "export": job})
                elif parsed.path == "/api/episodes":
                    self.send(list(reversed(list_episodes(controller.store.root))))
                elif parsed.path == "/api/episode":
                    path = episode_path(controller.store.root, params["id"])
                    with h5py.File(path, "r") as file:
                        values = {"state": file["state"][:].tolist(), "action": file["action"][:].tolist()}
                    self.send({**summary(path), **values, "vector_names": controller.store.metadata["vector_names"],
                               "quality": validate_episode(path, controller.store.metadata, decode_images=False)})
                elif parsed.path == "/api/frame":
                    path = episode_path(controller.store.root, params["id"])
                    role, index = params["role"], int(params["index"])
                    if role not in controller.cameras:
                        raise ValueError("unknown camera role")
                    with h5py.File(path, "r") as file:
                        if not 0 <= index < len(file["timestamp"]):
                            raise ValueError("frame outside episode")
                        self.send(np.asarray(file[f"images/{role}"][index]).tobytes(), "image/jpeg")
                elif parsed.path == "/api/preview":
                    role = params["role"]
                    if role not in controller.cameras:
                        raise ValueError("unknown camera role")
                    frame = controller.cameras[role].latest_before(time.monotonic())
                    if frame is None or time.monotonic() - frame.monotonic > .5:
                        self.send({"error": "camera unavailable"}, status=503)
                    else:
                        ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if not ok:
                            raise ValueError("preview encoding failed")
                        self.send(jpg.tobytes(), "image/jpeg")
                else:
                    name = "index.html" if parsed.path == "/" else parsed.path.removeprefix("/")
                    path = (STATIC / name).resolve()
                    if not path.is_relative_to(STATIC.resolve()) or not path.is_file():
                        self.send({"error": "not found"}, status=404)
                    else:
                        self.send(path.read_bytes(), mimetypes.guess_type(path)[0] or "application/octet-stream")
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (ValueError, KeyError, OSError) as exc:
                self.send({"error": str(exc)}, status=400)

        def do_POST(self):
            if (not self.check_host() or self.headers.get("X-Nero-Request") != "1"
                    or self.headers.get("Content-Type") != "application/json"):
                self.send({"error": "same-origin JSON request required"}, status=403)
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 16384:
                    raise ValueError("invalid request length")
                body = json.loads(self.rfile.read(size))
                if self.path == "/api/command":
                    command = body.pop("command")
                    with job_lock:
                        if export_job["state"] == "running":
                            raise ValueError("dataset export is running")
                        result = controller.command(command, **body)
                    self.send(result)
                elif self.path == "/api/export":
                    repo_id = body["repo_id"]
                    if not re.fullmatch(r"[A-Za-z0-9_-]+/[A-Za-z0-9_.-]+", repo_id):
                        raise ValueError("repo_id must be namespace/dataset")
                    with job_lock:
                        if export_job["state"] == "running" or controller.status()["state"] in ("recording", "saving"):
                            raise ValueError("finish the current recording/export first")
                        if controller.store.metadata["synthetic"] and body.get("allow_synthetic") is not True:
                            raise ValueError("synthetic export must be explicitly selected")
                        chosen = select_episodes(controller.store.root, body.get("episode_ids", []))
                        episode_ids = [info["id"] for info in chosen]
                        export_job.update(state="running", result=None, error=None,
                                          progress={"completed": 0, "total": len(chosen), "episode": None})
                    export_name = f"{repo_id.split('/')[1]}_{datetime.now():%Y%m%dT%H%M%S_%f}"
                    destination = controller.store.root / "exports" / export_name / repo_id

                    def run_export():
                        def progress(value):
                            with job_lock:
                                export_job["progress"] = value
                        try:
                            result = export_dataset(controller.store.root, destination, repo_id,
                                                    allow_synthetic=body.get("allow_synthetic", False),
                                                    episode_ids=episode_ids, progress=progress)
                            with job_lock:
                                export_job.update(state="complete", result=result)
                        except Exception as exc:
                            with job_lock:
                                export_job.update(state="error", error=str(exc))
                    pool.submit(run_export)
                    self.send({"started": True}, status=202)
                else:
                    self.send({"error": "not found"}, status=404)
            except Exception as exc:
                self.send({"error": str(exc)}, status=400)

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.export_pool = pool
    return server


def serve(controller, port):
    server = create_server(controller, port)
    print(f"NeroPicoData: http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        server.export_pool.shutdown(wait=True)
