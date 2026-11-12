#!/usr/bin/env python3
"""Read-only local dashboard for the running Go2 cruise simulation."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


class MonitorHandler(BaseHTTPRequestHandler):
    data_dir = None
    page = None

    def do_GET(self):
        route = urlparse(self.path).path
        if route == "/api/state":
            self._send_file(self.data_dir / "data.json", "application/json; charset=utf-8")
        elif route == "/api/map":
            self._send_file(self.data_dir / "occupancy_overlay.png", "image/png")
        elif route == "/":
            self._send_bytes(self.page, "text/html; charset=utf-8")
        else:
            self.send_error(404)

    def _send_file(self, path, content_type):
        try:
            content = path.read_bytes()
        except OSError:
            self.send_error(404, "Simulation output is not available yet")
            return
        self._send_bytes(content, content_type)

    def _send_bytes(self, content, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, fmt, *args):
        return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--page", default=str(Path(__file__).with_name("live_cruise_monitor.html")), type=Path)
    args = parser.parse_args()

    MonitorHandler.data_dir = args.data_dir.resolve()
    MonitorHandler.page = args.page.read_bytes()
    server = ThreadingHTTPServer((args.host, args.port), MonitorHandler)
    print(f"Cruise monitor: http://{args.host}:{args.port}/", flush=True)
    print(f"Reading simulation output from {MonitorHandler.data_dir}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
