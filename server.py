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
    """Métriques FHE estimées (mode simulation sans binaire)."""
    ss = 6   # batching : 6 SS constants
    keygen_ms = 45_000
    calc_ms   = ss * 820 + n_seg * 50
    ct_kb     = round(10 * 8.0, 1)
    return {"scheme_switches": ss,
            "ciphertext_size_kb": ct_kb,
            "time_ms": round(keygen_ms + calc_ms, 1)}

# ── Lancement du  binaire ─────────────────────────────────────────────────

def run_binary(paths):
    """
    Écrit les trajectoires dans des fichiers temporaires et lance drone_fhe.
    paths = liste de 2 ou 3 trajectoires [[x,y,z],...]
    """
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

        print(f"[server] Lancement FHE : {len(paths)} drones")
        t0 = time.time()

        result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        elapsed_ms = (time.time() - t0) * 1000

        if result.returncode != 0:
            return None, f"Erreur binaire (code {result.returncode}):\n{result.stderr[:500]}"

        # Parser JSON_RESULT
        for line in result.stdout.splitlines():
            if line.strip().startswith("JSON_RESULT:"):
                try:
                    data = json.loads(line.strip()[len("JSON_RESULT:"):])
                    data["mode"] = "fhe_reel"
                    data["total_time_ms"] = round(elapsed_ms, 1)
                    # Ajouter taille ciphertexts estimée
                    if "ciphertext_size_kb" not in data:
                        data["ciphertext_size_kb"] = 80.0
                    print(f"[server] FHE terminé en {elapsed_ms/1000:.1f}s — collision={data.get('collision')}")
                    return data, None
                except json.JSONDecodeError as e:
                    return None, f"JSON invalide : {e}"

        return None, f"Pas de JSON_RESULT.\nSortie:\n{result.stdout[-500:]}"

    except subprocess.TimeoutExpired:
        return None, "Timeout : calcul > 10 min."
    except Exception as e:
        return None, str(e)
    finally:
        for f in tmp_files:
            try: os.unlink(f)
            except: pass

def analyze_paths(paths):
    """
    Si binaire présent → FHE réel.
    Sinon → simulation Python instantanée.
    """
    # Détection en clair (toujours faite pour référence)
    names = [f"Drone {i+1}" for i in range(len(paths))]
    all_hits = []
    total_collisions = 0
    n_seg_total = 0

    for i in range(len(paths)):
        for j in range(i+1, len(paths)):
            hits = clear_intersection(paths[i], paths[j])
            n_seg_total += (len(paths[i])-1) * (len(paths[j])-1)
            if hits:
                total_collisions += 1
                for h in hits:
                    all_hits.append({
                        "pair": f"{names[i]} x {names[j]}",
                        "seg1": h["seg1"], "seg2": h["seg2"]
                    })

    if os.path.isfile(BINARY_PATH):
        data, error = run_binary(paths)
        if data:
            # Enrichir avec les détails en clair
            if "details" not in data or not data["details"]:
                data["details"] = all_hits
            return data
        else:
            print(f"[server] Binaire échoué : {error} — fallback simulation")

    # Mode simulation
    metrics = simulate_metrics(n_seg_total)
    return {
        "mode":               "simulation",
        "collision":          total_collisions > 0,
        "collisions_count":   total_collisions,
        "pairs_tested":       len(paths) * (len(paths)-1) // 2,
        "drones":             len(paths),
        "segments_tested":    n_seg_total,
        "scheme_switches":    metrics["scheme_switches"],
        "ciphertext_size_kb": metrics["ciphertext_size_kb"],
        "time_ms":            metrics["time_ms"],
        "details":            all_hits,
    }

# ── Handler HTTP ──────────────────────────────────────────────────────────────

class Handler(http.server.SimpleHTTPRequestHandler):

    def do_OPTIONS(self):
        self.send_response(200); self._cors(); self.end_headers()

    def do_GET(self):
        if urlparse(self.path).path == "/api/health":
            self._json({"status": "ok", "binary": os.path.isfile(BINARY_PATH)})
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

                print(f"[server] Analyse : {len(paths)} drones")
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
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def log_message(self, fmt, *args):
        print(f"[http] {fmt % args}")

# ── Serveur ───────────────────────────────────────────────────────────────────

class ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads      = True

if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    binary_ok = os.path.isfile(BINARY_PATH)
    print(f"  Drone HE — Serveur 3 drones + FHE batching   ")
    print(f" URL    : http://localhost:{PORT}                  ")
    print(f" Demo   : http://localhost:{PORT}/web/demo.html      ")
    print(f" Binaire: {'OK - FHE reel ' if binary_ok else 'absent (simulation)'}             ║")
    print(f"\n")

    with ThreadedServer(("", PORT), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServeur arrete.")
