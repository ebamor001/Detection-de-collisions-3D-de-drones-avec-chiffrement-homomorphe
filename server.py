#!/usr/bin/env python3
"""
server.py — Drone HE, lance le vrai binaire FHE avec batching.
python3 server.py
"""

import http.server
import json
import os
import subprocess
import socketserver
import tempfile
import time
from urllib.parse import urlparse

PORT        = 8765
BINARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")


def simulate_metrics(n_seg):
    ss = 6
    keygen_ms = 45_000
    calc_ms   = ss * 820 + n_seg * 50
    ct_kb     = round(10 * 8.0, 1)
    return {
        "scheme_switches": ss,
        "ciphertext_size_kb": ct_kb,
        "time_ms": round(keygen_ms + calc_ms, 1)
    }


class Handler(http.server.SimpleHTTPRequestHandler):

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path).path

        if parsed == "/api/health":
            self._json({
                "status": "ok",
                "binary": os.path.isfile(BINARY_PATH)
            })

        elif parsed == "/api/drone_status":
            state_file = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "drone_state.json"
            )

            if os.path.isfile(state_file):
                try:
                    with open(state_file, "r") as f:
                        self._json(json.load(f))
                except Exception as e:
                    self._json({"error": str(e)}, 500)
            else:
                self._json({"error": "Simulation non demarree"}, 404)

        else:
            super().do_GET()

    def do_POST(self):
        if urlparse(self.path).path == "/api/analyze":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)

            try:
                data  = json.loads(body)
                path1 = data.get("path1", [])
                path2 = data.get("path2", [])
                path3 = data.get("path3", [])

                if len(path1) < 2 or len(path2) < 2:
                    self._json({"error": "Minimum 2 points par drone"}, 400)
                    return

                paths = [path1, path2]
                if len(path3) >= 2:
                    paths.append(path3)

                result = analyze_paths(paths)
                self._json(result)

            except json.JSONDecodeError:
                self._json({"error": "JSON invalide"}, 400)
            except Exception as e:
                self._json({"error": str(e)}, 500)

        else:
            self._json({"error": "Not found"}, 404)

    def _json(self, data, code=200):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def log_message(self, fmt, *args):
        print(f"[http] {fmt % args}")


class ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads      = True


def run_binary(paths):
    tmp_files = []
    try:
        for i, path in enumerate(paths):
            with tempfile.NamedTemporaryFile(mode='w', suffix='.txt',
                                             delete=False, prefix=f'drone{i+1}_') as f:
                line = "".join(
                    f"({int(p[0])},{int(p[1])},{int(p[2]) if len(p)>2 else 0})"
                    for p in path
                )
                f.write(line + "\n")
                tmp_files.append(f.name)

        cmd = [BINARY_PATH, "--path1", tmp_files[0], "--path2", tmp_files[1]]
        if len(tmp_files) >= 3:
            cmd += ["--path3", tmp_files[2]]

        t0 = time.time()
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        elapsed_ms = (time.time() - t0) * 1000

        if result.returncode != 0:
            return None, result.stderr

        for line in result.stdout.splitlines():
            if line.startswith("JSON_RESULT:"):
                data = json.loads(line[len("JSON_RESULT:"):])
                data["mode"] = "fhe_reel"
                data["total_time_ms"] = round(elapsed_ms, 1)
                return data, None

        return None, "Pas de JSON_RESULT"

    finally:
        for f in tmp_files:
            try:
                os.unlink(f)
            except:
                pass


def analyze_paths(paths):
    total_collisions = 0
    n_seg_total = 0
    all_hits = []

    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            n_seg_total += (len(paths[i]) - 1) * (len(paths[j]) - 1)

    if os.path.isfile(BINARY_PATH):
        data, error = run_binary(paths)
        if data:
            return data

    metrics = simulate_metrics(n_seg_total)
    return {
        "mode": "simulation",
        "collision": False,
        "scheme_switches": metrics["scheme_switches"],
        "ciphertext_size_kb": metrics["ciphertext_size_kb"],
        "time_ms": metrics["time_ms"],
    }


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print(f"Drone HE server running on http://localhost:{PORT}")

    with ThreadedServer(("", PORT), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServeur arrete.")
