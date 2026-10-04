"""The rider app's HTTP API: JSON requests, trip updates as server-sent events.

Standard library only (like guide_ui's server), so it runs anywhere Python does.

  GET  /api/health
  GET  /api/car
  GET  /api/destinations?q=9896
  GET  /api/trips/current
  POST /api/trips                 {"destination": "ASB 9896", "pickup": "ASB 9700"}   pickup optional
  POST /api/trips/<id>/start      the rider met the car: begin guiding
  POST /api/trips/<id>/confirm    the rider did what the car asked
  POST /api/trips/<id>/pause      {"paused": true}
  POST /api/trips/<id>/cancel
  POST /api/car/position          {"room": "ASB 970.7", "headingDeg": 90}   only for a car that cannot localize
  GET  /api/trips/<id>/events     text/event-stream, one JSON event per change

Anything else is served from ``web/`` (the browser client, a test page).
"""

from __future__ import annotations

import json
import mimetypes
import queue
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from .service import ENDED, ApiError, ScoutService

WEB = Path(__file__).resolve().parent / "web"
TRIP = re.compile(r"^/api/trips/([\w-]+)/(start|confirm|pause|cancel|events)$")
KEEPALIVE_S = 10.0


def make_handler(service: ScoutService, token: str = ""):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        # The app may be served from somewhere else (a dev server, a phone wrapper).
        def _cors(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")

        def _send(self, code: int, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self._cors()
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200):
            self._send(code, json.dumps(obj).encode(), "application/json")

        def _error(self, e: ApiError):
            self._json({"error": e.code, "message": e.message}, e.status)

        def _allowed(self) -> bool:
            if not token or self.headers.get("Authorization", "") == f"Bearer {token}":
                return True
            self._json({"error": "unauthorized", "message": "missing or wrong token"}, 401)
            return False

        def _body(self) -> dict:
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0:
                return {}
            if n > 65536:
                raise ApiError(413, "too_large", "request body too large")
            try:
                doc = json.loads(self.rfile.read(n))
            except ValueError:
                raise ApiError(400, "bad_json", "the body is not JSON") from None
            if not isinstance(doc, dict):
                raise ApiError(400, "bad_json", "the body must be a JSON object")
            return doc

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors()
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            url = urlparse(self.path)
            path = url.path
            try:
                if path == "/api/health":
                    return self._json(service.health())
                if path == "/api/car":
                    return self._json(service.car())
                if path == "/api/destinations":
                    q = parse_qs(url.query)
                    return self._json(service.destination_list(q.get("q", [""])[0], int(q.get("limit", ["40"])[0])))
                if path == "/api/trips/current":
                    t = service.trip
                    return self._json(service.snapshot(t, with_route=True) if t else None)
                m = TRIP.match(path)
                if m and m.group(2) == "events":
                    # EventSource cannot set headers, so the token may also come as ?token=
                    if token and parse_qs(url.query).get("token", [""])[0] != token and not self._allowed():
                        return None
                    return self._events(m.group(1))
            except ApiError as e:
                return self._error(e)
            except ValueError:
                return self._json({"error": "bad_request", "message": "bad query"}, 400)
            name = "index.html" if path in ("", "/") else path.lstrip("/")
            target = (WEB / name).resolve()
            if WEB not in target.parents or not target.is_file():
                return self._send(404, b"not found", "text/plain")
            return self._send(200, target.read_bytes(), mimetypes.guess_type(str(target))[0] or "application/octet-stream")

        def do_POST(self):
            path = urlparse(self.path).path
            if not self._allowed():
                return None
            try:
                body = self._body()
                if path == "/api/car/position":
                    return self._json(service.relocate(body.get("room", ""), body.get("headingDeg")))
                if path == "/api/trips":
                    return self._json(service.request_trip(body.get("destination", ""), body.get("pickup") or None), 201)
                m = TRIP.match(path)
                if m:
                    trip_id, verb = m.groups()
                    if verb == "start":
                        return self._json(service.start_guiding(trip_id))
                    if verb == "confirm":
                        return self._json(service.confirm(trip_id))
                    if verb == "pause":
                        return self._json(service.pause(trip_id, bool(body.get("paused", True))))
                    if verb == "cancel":
                        return self._json(service.cancel(trip_id))
            except ApiError as e:
                return self._error(e)
            return self._json({"error": "not_found", "message": "no such endpoint"}, 404)

        def _events(self, trip_id: str):
            q, first = service.subscribe(trip_id)  # raises ApiError for an unknown trip, before any header is sent
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self._cors()
            self.end_headers()
            self.close_connection = True
            try:
                ev = first
                while True:
                    self.wfile.write(b"data: " + json.dumps(ev).encode() + b"\n\n")
                    self.wfile.flush()
                    if ev["phase"] in ENDED:
                        break
                    while True:
                        try:
                            ev = q.get(timeout=KEEPALIVE_S)
                            break
                        except queue.Empty:
                            self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
            except OSError:
                pass  # the app went away
            finally:
                service.unsubscribe(q)

        def log_message(self, *a):
            pass

    return Handler


def serve(service: ScoutService, port: int = 8090, host: str = "0.0.0.0", token: str = "") -> ThreadingHTTPServer:
    """Start the server on a daemon thread and return it (``.shutdown()`` to stop)."""
    server = ThreadingHTTPServer((host, port), make_handler(service, token))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
