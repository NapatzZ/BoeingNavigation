#!/usr/bin/env python3
"""
Map editor: a small web app to clean up saved maps and place named locations.

    ros2 run skuba_navigation map_editor            # http://<robot>:8090
    ros2 run skuba_navigation map_editor --port 9000

Edits maps/<category>/<name>.pgm in this package's source checkout (original
kept as <name>.pgm.bak) and the selected map's <name>.poses.yaml (timestamped
.bak), the same files map_save_cli and save_location write. Not a ROS node: it only
reads and writes files, so it works with or without a running stack.

HTTP glue only; the file/coordinate logic is in core/map_editor.py.
"""

import argparse
import base64
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from ament_index_python.packages import get_package_share_directory

from ..core import map_editor as editor
from ..core.package_paths import find_package_source_dir

PACKAGE_NAME = "skuba_navigation"


def resolve_paths():
    """Return (maps root, editor html), source checkout first."""
    share = Path(get_package_share_directory(PACKAGE_NAME))
    source = find_package_source_dir(share, PACKAGE_NAME)
    base = source if source is not None else share
    html = base / "web" / "map_editor.html"
    if not html.exists():
        html = share / "web" / "map_editor.html"
    return base / "maps", html


def make_handler(maps_root, html_path):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # keep the terminal quiet; errors are returned as JSON

        def _send(self, status, body, content_type="application/json"):
            if not isinstance(body, bytes):
                body = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _body(self):
            length = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(length) or b"{}")

        def _guard(self, fn):
            try:
                self._send(200, fn())
            except (ValueError, KeyError, FileNotFoundError) as exc:
                self._send(400, {"error": str(exc)})
            except Exception as exc:  # report, never kill the server
                self._send(500, {"error": f"{type(exc).__name__}: {exc}"})

        def do_GET(self):
            url = urlparse(self.path)
            query = parse_qs(url.query)
            if url.path in ("/", "/index.html"):
                self._send(200, html_path.read_bytes(), "text/html; charset=utf-8")
            elif url.path == "/api/maps":
                self._guard(lambda: {"maps": editor.list_maps(maps_root)})
            elif url.path == "/api/map":
                self._guard(lambda: self._map(query["id"][0]))
            elif url.path == "/api/locations":
                self._guard(lambda: self._locations(query["id"][0]))
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            url = urlparse(self.path)
            if url.path == "/api/map":
                self._guard(self._save_map)
            elif url.path == "/api/locations":
                self._guard(self._save_locations)
            else:
                self._send(404, {"error": "not found"})

        def _map(self, map_id):
            data = editor.load_map(maps_root, map_id)
            data["pixels"] = base64.b64encode(bytes(data["pixels"])).decode()
            return data

        def _locations(self, map_id):
            path = editor.poses_path(maps_root, map_id)
            return {"path": str(path), "locations": editor.load_locations(path)}

        def _save_locations(self):
            body = self._body()
            path = editor.poses_path(maps_root, body["id"])
            return {"saved": editor.save_locations(path, body["locations"])}

        def _save_map(self):
            body = self._body()
            pixels = base64.b64decode(body["pixels"])
            saved = editor.save_map_pixels(
                maps_root, body["id"], int(body["width"]), int(body["height"]), pixels
            )
            return {"saved": saved}

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    args, _ = parser.parse_known_args()  # ignore --ros-args from ros2 run

    maps_root, html_path = resolve_paths()
    server = ThreadingHTTPServer(
        (args.host, args.port), make_handler(maps_root, html_path)
    )
    print(f"Map editor: http://{args.host}:{args.port}")
    print(f"  maps:      {maps_root}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
