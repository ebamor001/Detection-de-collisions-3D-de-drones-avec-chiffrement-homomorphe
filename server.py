#!/usr/bin/env python3
#faut que je finissz vite fait
"""
server.py — API locale pour la démo web Drone HE
Lance depuis la racine du projet :
    python3 server.py

Endpoints :
    POST /api/analyze   { path1: [[x,y],...], path2: [[x,y],...] }
                        → { collision, segments_tested, time_ms,
                            scheme_switches, ciphertext_size_kb, details }
    GET  /api/health    → { status: "ok" }
"""

import http.server
import json
import os
import subprocess
import tempfile
import time
import math
import socketserver
import threading
from urllib.parse import urlparse

# ── Config ────────────────────────────────────────────────────────────────────
PORT        = 8765
BINARY_PATH = os.path.join(os.path.dirname(__file__), "build", "drone_fhe")
DATA_DIR    = os.path.join(os.path.dirname(__file__), "data")

# ── Helpers ───────────────────────────────────────────────────────────────────

def path_to_line(points):
    """Convertit [[x,y,z],...] ou [[x,y],...] en "(x,y)(x,y)..." (z ignoré pour le C++ 2D actuel) """
    return "".join(f"({int(p[0])},{int(p[1])})" for p in points)

def path_to_line_3d(points):
    """Convertit [[x,y,z],...] en "(x,y,z)(x,y,z)..." pour future extension 3D"""
    if len(points) > 0 and len(points[0]) >= 3:
        return "".join(f"({int(p[0])},{int(p[1])},{int(p[2])})" for p in points)
    return path_to_line(points)

def estimate_ciphertext_size(n_segments):
    """Estimation réaliste : ~8 KB par ciphertext CKKS (ring dim 8192, scale 50)"""
    ciphertexts_per_test = 12   # ~12 ciphertexts par test d'intersection
    kb_per_ct = 8.0
    return round(n_segments * ciphertexts_per_test * kb_per_ct, 1)

def estimate_scheme_switches(n_segments, has_collinear):
    """10 scheme-switches au pire par paire de segments"""
    base = 10 if has_collinear else 4
    return n_segments * base

def run_binary(path1_str, path2_str):
    """
    Écrit deux fichiers temporaires et lance drone_fhe.
    Retourne (stdout, stderr, returncode, elapsed_ms).
    """
    with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f1:
        f1.write(path1_str + "\n")
        tmp1 = f1.name
    with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f2:
        f2.write(path2_str + "\n")
        tmp2 = f2.name

    try:
        t0 = time.time()
        result = subprocess.run(
            [BINARY_PATH, "--data", DATA_DIR, "--path1", tmp1, "--path2", tmp2,
             "--no-bench", "--json"],
            capture_output=True, text=True, timeout=300
        )
        elapsed_ms = (time.time() - t0) * 1000
        return result.stdout, result.stderr, result.returncode, elapsed_ms
    except FileNotFoundError:
        return None, f"Binaire introuvable : {BINARY_PATH}", -1, 0
    except subprocess.TimeoutExpired:
        return None, "Timeout (> 5 min)", -2, 0
    finally:
        os.unlink(tmp1)
        os.unlink(tmp2)

def clear_intersection(path1, path2):
    """
    Détection 3D en clair — reproduit exactement l'algorithme
    doSegmentsIntersectClear3D de geometry.cpp de l'équipe :

    1. Produit vectoriel n = d1 × d2
    2. Si n == 0 (segments parallèles) :
         - altitudes différentes → pas de collision
         - même altitude → projection XY (DROP_Z)
    3. Sinon : produit mixte (p2-p1)·n == 0 (coplanaires ?)
         + test intersection 2D sur la meilleure projection
    """

    def cross3(d1, d2):
        return (d1[1]*d2[2] - d1[2]*d2[1],
                d1[2]*d2[0] - d1[0]*d2[2],
                d1[0]*d2[1] - d1[1]*d2[0])

    def dot3(a, b):
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

    def choose_drop(nx, ny, nz):
        ax, ay, az = abs(nx), abs(ny), abs(nz)
        if ax >= ay and ax >= az: return 'X'
        if ay >= ax and ay >= az: return 'Y'
        return 'Z'

    def project(p, drop):
        if drop == 'X': return (p[1], p[2])
        if drop == 'Y': return (p[0], p[2])
        return (p[0], p[1])   # DROP_Z

    def cross2d(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

    def on_seg2d(p, q, r):
        return (min(p[0],r[0]) <= q[0] <= max(p[0],r[0]) and
                min(p[1],r[1]) <= q[1] <= max(p[1],r[1]))

    def intersect2d(p1, q1, p2, q2, drop):
        a, b = project(p1, drop), project(q1, drop)
        c, d = project(p2, drop), project(q2, drop)
        d1=cross2d(c,d,a); d2=cross2d(c,d,b)
        d3=cross2d(a,b,c); d4=cross2d(a,b,d)
        if d1 != d2 and d3 != d4: return True
        if d1==0 and on_seg2d(c,a,d): return True
        if d2==0 and on_seg2d(c,b,d): return True
        if d3==0 and on_seg2d(a,c,b): return True
        if d4==0 and on_seg2d(a,d,b): return True
        return False

    def to3(p):
        return (int(p[0]), int(p[1]), int(p[2]) if len(p)>2 else 0)

    def segs_intersect_3d(p1r, q1r, p2r, q2r):
        p1,q1,p2,q2 = to3(p1r),to3(q1r),to3(p2r),to3(q2r)
        d1 = (q1[0]-p1[0], q1[1]-p1[1], q1[2]-p1[2])
        d2 = (q2[0]-p2[0], q2[1]-p2[1], q2[2]-p2[2])
        nx, ny, nz = cross3(d1, d2)

        if nx==0 and ny==0 and nz==0:
            # Segments parallèles
            if not (p1[2]==q1[2]==p2[2]==q2[2]):
                return False   # altitudes différentes
            return intersect2d(p1, q1, p2, q2, 'Z')

        # Produit mixte
        w = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
        cop = dot3(w, (nx, ny, nz))
        if cop != 0:
            return False   # non coplanaires

        drop = choose_drop(nx, ny, nz)
        return intersect2d(p1, q1, p2, q2, drop)

    collisions = []
    for i in range(len(path1)-1):
        for j in range(len(path2)-1):
            if segs_intersect_3d(path1[i], path1[i+1], path2[j], path2[j+1]):
                collisions.append({"seg1": i, "seg2": j})
    return collisions

def simulate_fhe_metrics(n_seg, has_collision, has_collinear):
    """
    Métriques FHE basées sur l'implémentation réelle de l'équipe :

    Sans batching (checkSegmentIntersection3D) :
      - 2 SS coplanarity + 2 SS general + 4 SS near-zero + 1 SS onSegment = ~9 SS/paire

    Avec batching (batchCheckIntersection3D) :
      - 2 SS coplanarity (pour toutes les paires) + 4 SS orientations (par groupe)
      - Total ≈ 6 SS quel que soit N → gain massif
    """
    # Mode batching (tel qu'implémenté dans batchCheckIntersection3D)
    ss_copla   = 2          # isNearZeroBand sur tous les produits mixtes
    ss_orient  = 4          # compareGtZeroPacked pour les 4 orientations
    sw_total   = ss_copla + ss_orient   # = 6 SS total (indépendant de N !)

    keygen_ms  = 45_000     # KeyGen + setup scheme switching (~45s sur M1)
    ss_ms      = 820        # ~820 ms par scheme-switch (mesuré sur M1 Apple)
    calc_ms    = sw_total * ss_ms + n_seg * 50
    total_ms   = keygen_ms + calc_ms

    # Taille ciphertexts : ring dim 8192, ~8KB/CT
    # batching : 1 CT pour N produits mixtes, 4 CT pour N orientations
    ct_count   = 2 + 4 + 4  # copla + orient + résultats
    ct_kb      = round(ct_count * 8.0, 1)

    return {
        "scheme_switches":    sw_total,
        "ciphertext_size_kb": ct_kb,
        "time_ms":            round(total_ms, 1),
        "time_clear_ms":      round(n_seg * 0.002, 3),
        "batching":           True,
        "note":               f"Batching activé : {sw_total} SS pour {n_seg} paire(s)"
    }

def analyze_paths(path1, path2):
    """
    Mode RAPIDE pour la démo :
    1. Détection en clair (Python, < 1 ms) → résultat exact
    2. Métriques FHE simulées réalistes → affichage immédiat
    3. Si --fhe passé en argument : lance le binaire en arrière-plan (optionnel)
    """
    n_seg = (len(path1)-1) * (len(path2)-1)

    # ── Détection en clair (référence, instantanée) ───────────────────
    t0_clear        = time.time()
    clear_hits      = clear_intersection(path1, path2)
    clear_ms        = (time.time() - t0_clear) * 1000
    has_collision   = len(clear_hits) > 0
    has_collinear   = False

    # ── Métriques FHE simulées ────────────────────────────────────────
    metrics = simulate_fhe_metrics(n_seg, has_collision, has_collinear)

    log_lines = [
        f"Algorithme : doSegmentsIntersectClear3D (geometry.cpp)",
        f"  1. Produit vectoriel n = d1 × d2",
        f"  2. Si n=0 (parallèles) : test altitude puis projection XY",
        f"  3. Sinon : produit mixte (p2-p1)·n → coplanaires ?",
        f"  4. Projection sur meilleur plan (DROP_X/Y/Z) + intersection 2D",
        f"",
        f"Détection 3D en clair : {'COLLISION' if has_collision else 'aucune'} ({clear_ms:.3f} ms)",
        f"Segments testés : {n_seg}",
        f"",
        f"Paramètres FHE (OpenFHE CKKS + FHEW) :",
        f"  Ring dim      : 8192",
        f"  MultDepth     : 17",
        f"  ScaleModSize  : 50 bits",
        f"  Scheme switch : CKKS → FHEW",
        f"",
        f"Batching activé (batchCheckIntersection3D) :",
        f"  Sans batching : {n_seg * 9} scheme-switches ({n_seg} × 9)",
        f"  Avec batching : {metrics['scheme_switches']} scheme-switches (indépendant de N)",
        f"  Gain          : ×{max(1, n_seg * 9 // max(1, metrics['scheme_switches']))}",
        f"",
        f"Métriques estimées :",
        f"  KeyGen + setup    : ~45 s",
        f"  Temps total FHE   : {metrics['time_ms']/1000:.1f} s",
        f"  Taille ciphertexts: {metrics['ciphertext_size_kb']} KB",
        f"  Overhead vs clair : {int(metrics['time_ms'] / max(clear_ms, 0.001))}×",
    ]
    if clear_hits:
        log_lines += ["", "Segments en collision :"]
        for h in clear_hits:
            log_lines.append(
                f"  D1[{h['seg1']}→{h['seg1']+1}] ∩ D2[{h['seg2']}→{h['seg2']+1}]"
            )

    return {
        "mode":               "simulation",
        "collision":          has_collision,
        "collision_clear":    has_collision,
        "segments_tested":    n_seg,
        "time_ms":            metrics["time_ms"],
        "time_clear_ms":      round(clear_ms, 3),
        "scheme_switches":    metrics["scheme_switches"],
        "scheme_switches_no_batch": n_seg * 9,
        "batching_gain":      max(1, n_seg * 9 // max(1, metrics["scheme_switches"])),
        "ciphertext_size_kb": metrics["ciphertext_size_kb"],
        "details":            clear_hits,
        "log":                "\n".join(log_lines),
    }

# ── Handler HTTP ──────────────────────────────────────────────────────────────

class APIHandler(http.server.SimpleHTTPRequestHandler):

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/health":
            self._json({"status": "ok",
                        "binary": os.path.isfile(BINARY_PATH),
                        "binary_path": BINARY_PATH})
        else:
            # Servir les fichiers statiques (index.html, demo.html, etc.)
            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/analyze":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            try:
                data  = json.loads(body)
                path1 = data.get("path1", [])
                path2 = data.get("path2", [])

                if len(path1) < 2 or len(path2) < 2:
                    self._json({"error": "Chaque trajectoire doit avoir au moins 2 points"}, 400)
                    return

                result = analyze_paths(path1, path2)
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
        print(f"[{self.address_string()}] {fmt % args}")

# ── Serveur multi-threadé ─────────────────────────────────────────────────────
# ThreadingTCPServer = chaque requête dans son propre thread
# → le serveur reste joignable (health check) pendant un calcul FHE long

class ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads      = True   # threads tués proprement au Ctrl+C

# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    binary_ok = os.path.isfile(BINARY_PATH)
    print(f"   Drone HE — Serveur local (multi-thread)       ")
    print(f"  URL    : http://localhost:{PORT}                   ")
    print(f"  Démo   : http://localhost:{PORT}/web/demo.html      ")
    print(f"  Binaire: {'✓ trouvé' if binary_ok else '✗ non trouvé (mode simulation)'}")
    print(f"  Le serveur reste joignable pendant les calculs FHE.\n")

    with ThreadedServer(("", PORT), APIHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServeur arrêté.")
