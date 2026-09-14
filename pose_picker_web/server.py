#!/usr/bin/env python3
"""
pose_picker_web
===============

A tiny local web service to pick robot poses on a map by clicking.

Open the page in a browser, choose a map, then:
  * click a spot            -> sets the position
  * drag before releasing   -> sets the heading (direction the arrow points)
  * type a name             -> saves it into share/saved_pose.yaml

Saved poses are drawn on the map as green arrows with labels, in the exact
same YAML format used by save_location.py, so the rest of the stack keeps
working unchanged.

Run:
    python3 pose_picker_web/server.py
    # then open http://localhost:8080

No external dependencies except PyYAML (already used by the package).
Map images (.pgm) are parsed in pure Python and streamed to the browser as
raw grayscale, so nothing needs to be installed to view them.
"""

import argparse
import base64
import json
import math
import os
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import yaml

HERE = Path(__file__).resolve().parent
PKG_ROOT = HERE.parent                      
MAPS_DIR = PKG_ROOT / 'maps' / '01_test_skuba_lab_610'
YAML_PATH = PKG_ROOT / 'share' / 'saved_pose.yaml'
INDEX_HTML = HERE / 'index.html'


# --------------------------------------------------------------------------- #
# PGM parsing (P5 binary / P2 ascii), pure Python.
# --------------------------------------------------------------------------- #
def _read_pnm_tokens(data: bytes, start: int):
    """Yield whitespace-separated tokens from a PNM header, skipping comments."""
    i = start
    n = len(data)
    while i < n:
        c = data[i:i + 1]
        if c in b' \t\r\n':
            i += 1
            continue
        if c == b'#':
            while i < n and data[i:i + 1] not in b'\r\n':
                i += 1
            continue
        j = i
        while j < n and data[j:j + 1] not in b' \t\r\n#':
            j += 1
        yield data[i:j].decode('ascii'), j
        i = j


def parse_pgm(path: Path):
    """Return (width, height, maxval, grayscale_bytes) for a P5/P2 PGM."""
    raw = path.read_bytes()
    if raw[:2] not in (b'P5', b'P2'):
        raise ValueError(f'Not a PGM (P5/P2): {path}')
    magic = raw[:2]

    tokens = _read_pnm_tokens(raw, 2)
    width, pos = next(tokens)
    height, pos = next(tokens)
    maxval, pos = next(tokens)
    width, height, maxval = int(width), int(height), int(maxval)

    if magic == b'P5':
        # One whitespace byte separates the header from binary data.
        data_start = pos + 1
        pixels = raw[data_start:data_start + width * height]
        if maxval > 255:  # 16-bit: take high byte
            pixels = pixels[::2]
    else:  # P2 ascii
        vals = []
        for tok, _ in tokens:
            vals.append(int(tok))
            if len(vals) >= width * height:
                break
        pixels = bytes(min(255, v * 255 // maxval) for v in vals)

    return width, height, maxval, bytes(pixels)


# --------------------------------------------------------------------------- #
# Map metadata
# --------------------------------------------------------------------------- #
def list_maps():
    return sorted(p.stem for p in MAPS_DIR.glob('*.yaml'))


def load_map_meta(name: str):
    yaml_file = MAPS_DIR / f'{name}.yaml'
    meta = yaml.safe_load(yaml_file.read_text())
    image_ref = meta.get('image', f'{name}.pgm')
    image_path = (MAPS_DIR / image_ref).resolve()
    return {
        'name': name,
        'resolution': float(meta.get('resolution', 0.05)),
        'origin': [float(v) for v in meta.get('origin', [0.0, 0.0, 0.0])],
        'negate': int(meta.get('negate', 0)),
        'image_path': image_path,
    }


# --------------------------------------------------------------------------- #
# Saved-pose YAML
# --------------------------------------------------------------------------- #
def yaw_to_quat(yaw: float):
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}


def quat_to_yaw(q: dict):
    x, y = q.get('x', 0.0), q.get('y', 0.0)
    z, w = q.get('z', 0.0), q.get('w', 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_positions():
    if not YAML_PATH.exists():
        return {}
    data = yaml.safe_load(YAML_PATH.read_text()) or {}
    return data.get('positions', {}) or {}


def dump_positions(positions: dict):
    YAML_PATH.parent.mkdir(parents=True, exist_ok=True)
    YAML_PATH.write_text(
        yaml.safe_dump({'positions': positions}, default_flow_style=False))


def positions_as_list():
    out = []
    for name, entry in load_positions().items():
        pose = entry.get('pose', {})
        pos = pose.get('position', {})
        ori = pose.get('orientation', {})
        out.append({
            'name': name,
            'x': float(pos.get('x', 0.0)),
            'y': float(pos.get('y', 0.0)),
            'yaw': quat_to_yaw(ori),
            'side': entry.get('header', {}).get('side'),
        })
    return out


def save_pose(name, x, y, yaw):
    positions = load_positions()
    existing = positions.get(name, {})
    header = existing.get('header', {}) if isinstance(existing, dict) else {}
    header['frame_id'] = 'map'
    q = yaw_to_quat(yaw)
    positions[name] = {
        'header': header,
        'pose': {
            'position': {'x': float(x), 'y': float(y), 'z': 0.0},
            'orientation': {'x': q['x'], 'y': q['y'], 'z': q['z'], 'w': q['w']},
        },
    }
    dump_positions(positions)


def delete_pose(name):
    positions = load_positions()
    if name in positions:
        del positions[name]
        dump_positions(positions)
        return True
    return False


# --------------------------------------------------------------------------- #
# HTTP handler
# --------------------------------------------------------------------------- #
class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep the console quiet

    # -- helpers ----------------------------------------------------------- #
    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body_json(self):
        length = int(self.headers.get('Content-Length', 0))
        if not length:
            return {}
        return json.loads(self.rfile.read(length).decode('utf-8'))

    # -- routing ----------------------------------------------------------- #
    def do_GET(self):
        parsed = urlparse(self.path)
        route = parsed.path
        query = parse_qs(parsed.query)

        if route == '/':
            self._serve_index()
        elif route == '/api/maps':
            self._send_json({'maps': list_maps()})
        elif route == '/api/map':
            self._serve_map_meta(query.get('name', [''])[0])
        elif route == '/api/mapdata':
            self._serve_map_data(query.get('name', [''])[0])
        elif route == '/api/poses':
            self._send_json({'poses': positions_as_list()})
        else:
            self.send_error(404, 'Not found')

    def do_POST(self):
        route = urlparse(self.path).path
        try:
            body = self._read_body_json()
        except Exception as e:
            self._send_json({'error': f'bad json: {e}'}, 400)
            return

        if route == '/api/poses':
            name = str(body.get('name', '')).strip()
            if not name:
                self._send_json({'error': 'name required'}, 400)
                return
            save_pose(name, body['x'], body['y'], body['yaw'])
            self._send_json({'ok': True, 'poses': positions_as_list()})
        elif route == '/api/poses/delete':
            name = str(body.get('name', '')).strip()
            ok = delete_pose(name)
            self._send_json({'ok': ok, 'poses': positions_as_list()})
        else:
            self.send_error(404, 'Not found')

    # -- endpoints --------------------------------------------------------- #
    def _serve_index(self):
        html = INDEX_HTML.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _serve_map_meta(self, name):
        if name not in list_maps():
            self._send_json({'error': 'unknown map'}, 404)
            return
        meta = load_map_meta(name)
        try:
            w, h, _, _ = parse_pgm(meta['image_path'])
        except Exception as e:
            self._send_json({'error': f'image parse failed: {e}'}, 500)
            return
        self._send_json({
            'name': name,
            'resolution': meta['resolution'],
            'origin': meta['origin'],
            'width': w,
            'height': h,
        })

    def _serve_map_data(self, name):
        if name not in list_maps():
            self._send_json({'error': 'unknown map'}, 404)
            return
        meta = load_map_meta(name)
        try:
            w, h, _, pixels = parse_pgm(meta['image_path'])
        except Exception as e:
            self._send_json({'error': f'image parse failed: {e}'}, 500)
            return
        self._send_json({
            'width': w,
            'height': h,
            'pixels': base64.b64encode(pixels).decode('ascii'),
        })


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print('pose_picker_web running')
    print(f'  maps dir : {MAPS_DIR}')
    print(f'  saving to: {YAML_PATH}')
    print(f'  open     : http://{args.host}:{args.port}\n')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nstopped.')


if __name__ == '__main__':
    main()
