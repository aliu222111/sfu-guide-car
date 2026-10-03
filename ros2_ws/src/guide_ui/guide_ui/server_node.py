"""Serves the phone web app on campus WiFi (spec section 9).

Plain HTML and JavaScript, no build step. Live data goes over rosbridge
(``rosbridge_server``, WebSocket port 9090); this node serves the static files
and map data.
"""

from __future__ import annotations

import json
import mimetypes
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node

from .api import MapData

WEB = Path(__file__).resolve().parent / "web"


def make_handler(data: MapData, config: dict):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(), "application/json")

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/config":
                return self._json(config)
            if path == "/api/destinations":
                return self._json(data.destination_list())
            if path.startswith("/api/floor/"):
                f = data.floor(path.rsplit("/", 1)[1])
                return self._json(f) if f else self._json({"error": "unknown floor"}, 404)
            name = "index.html" if path in ("", "/") else path.lstrip("/")
            target = (WEB / name).resolve()
            if WEB not in target.parents or not target.is_file():
                return self._send(404, b"not found", "text/plain")
            ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
            return self._send(200, target.read_bytes(), ctype)

        def log_message(self, *a):
            pass

    return Handler


class UiServer(Node):
    def __init__(self):
        super().__init__("ui_server")
        maps_dir = self.declare_parameter("maps_dir", "maps").value
        port = self.declare_parameter("port", 8080).value
        config = {"rosbridge_port": self.declare_parameter("rosbridge_port", 9090).value, "heartbeat_s": 1.0}
        self.server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(MapData(maps_dir), config))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f"phone app on http://<this laptop>:{port}/")


def main(args=None):
    rclpy.init(args=args)
    node = UiServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
