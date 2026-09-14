"""Compute-only IK worker; no arm, CAN connection or PICO SDK is passed to it."""

import multiprocessing
import signal
import time

import numpy as np

from .nero_relative_core import NeroIK, NeroKinematics


def _serve(connection, config):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        if config.get("ik_solver") == "differential":
            from .nero_differential_ik import NeroDifferentialIK
            solver = NeroDifferentialIK(config)
        else:
            solver = NeroIK(config)
        connection.send({"kind": "ready"})
        while True:
            request = connection.recv()
            if request is None:
                return
            request_id, arguments = request
            result = solver.solve(**arguments)
            connection.send({"kind": "result", "id": request_id, "result": result})
    except EOFError:
        pass
    except Exception as exc:
        try:
            connection.send({"kind": "error", "error": str(exc)})
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        connection.close()


class ProcessNeroIK(NeroKinematics):
    """Keep parent FK local and give blocking optimization a separate interpreter."""

    def __init__(self, config, *, stopping=lambda: False):
        super().__init__(config)
        self.stopping = stopping
        self.process = self.connection = None
        self.request_id = 0
        self.failed = False
        self.solve_timeout_s = min(.15, config["max_gap_s"] * .75)

    def __enter__(self):
        if self.process is not None:
            raise RuntimeError("IK process is already started")
        # spawn avoids inheriting CAN sockets, SDK threads or the PICO library.
        context = multiprocessing.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_serve, args=(child, self.config),
                                       name="nero-ik", daemon=True)
        try:
            self.process.start()
            child.close()
            if self._receive(30., "initialization").get("kind") != "ready":
                raise RuntimeError("unexpected IK initialization response")
            return self
        except BaseException:
            child.close()
            self.close()
            raise

    def _receive(self, timeout, phase):
        deadline = time.monotonic() + timeout
        try:
            while True:
                if self.stopping():
                    raise RuntimeError(f"IK {phase} cancelled")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f"IK process {phase} timed out after {timeout * 1000:.0f} ms")
                # Connection.poll releases the GIL while CAN and PICO threads run.
                if self.connection.poll(min(.005, remaining)):
                    response = self.connection.recv()
                    if self.stopping():
                        raise RuntimeError(f"IK {phase} cancelled")
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f"IK process {phase} timed out after {timeout * 1000:.0f} ms")
                    if response.get("kind") == "error":
                        raise RuntimeError(f"IK process failed: {response['error']}")
                    return response
                if not self.process.is_alive():
                    raise RuntimeError(f"IK process exited during {phase}")
        except (EOFError, OSError) as exc:
            self.failed = True
            raise RuntimeError(f"IK process connection failed during {phase}: {exc}") from exc
        except RuntimeError:
            self.failed = True
            raise

    def solve(self, position, rotation, previous, session_seed, dt):
        if self.failed or self.process is None or not self.process.is_alive():
            raise RuntimeError("IK process is unavailable; no automatic restart")
        started = time.monotonic()
        if self.stopping():
            self.failed = True
            raise RuntimeError("IK solve cancelled")
        self.request_id += 1
        arguments = {"position": np.asarray(position).tolist(),
                     "rotation": np.asarray(rotation).tolist(),
                     "previous": np.asarray(previous).tolist(),
                     "session_seed": np.asarray(session_seed).tolist(), "dt": float(dt)}
        try:
            self.connection.send((self.request_id, arguments))
        except (BrokenPipeError, EOFError, OSError) as exc:
            self.failed = True
            raise RuntimeError(f"IK request failed: {exc}") from exc
        response = self._receive(self.solve_timeout_s, "solve")
        if response.get("kind") != "result" or response.get("id") != self.request_id:
            self.failed = True
            raise RuntimeError("IK response does not match the pending request")
        result = response["result"]
        return {**result, "worker_solve_ms": result.get("solve_ms"),
                "solve_ms": (time.monotonic() - started) * 1000}

    def close(self):
        process = self.process
        try:
            if process is not None and process.pid is not None:
                if process.is_alive():
                    try:
                        self.connection.send(None)
                    except (BrokenPipeError, EOFError, OSError):
                        pass
                process.join(timeout=.2)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=.5)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=.5)
                if process.is_alive():
                    raise RuntimeError("IK process did not exit")
                process.close()
        finally:
            if self.connection is not None:
                self.connection.close()
            self.connection = self.process = None

    def __exit__(self, *_args):
        self.close()
