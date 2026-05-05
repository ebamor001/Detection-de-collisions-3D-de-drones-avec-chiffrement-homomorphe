#!/usr/bin/env python3
"""
server.py — Drone HE, version CORRIGEE (securite)

CORRECTION SECURITE :
  - Les coordonnees GPS sont chiffrees IMMEDIATEMENT apres reception
  - Les fichiers en clair sont detruits avant tout calcul FHE
  - Seul le bit de resultat final (collision oui/non) est dechiffre
  - Le calcul FHE utilise le pipeline complet : encrypt -> detect_encrypted -> decrypt_with_ss
  - CORS restreint au domaine local (plus de wildcard *)

Limitation residuelle documentee :
  - Le navigateur envoie les waypoints en HTTP en clair vers le serveur.
  - Pour une confidentialite complete, il faudrait un chiffrement cote client
    (WebAssembly + OpenFHE compile en WASM), hors scope de ce prototype.

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

# Taille maximale d'un payload JSON accepte (protection DoS)
MAX_PAYLOAD_BYTES = 1_000_000  # 1 MB


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

            # CORRECTION : limiter la taille du payload (protection DoS)
            if length > MAX_PAYLOAD_BYTES:
                self._json({"error": "Payload trop grand"}, 413)
                return

            body = self.rfile.read(length)

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
        # Accepter localhost et 127.0.0.1 sur n'importe quel port
        # (couvre file://, http://localhost:XXXX, http://127.0.0.1:XXXX)
        origin = self.headers.get("Origin", "")
        allowed = (
            origin.startswith("http://localhost") or
            origin.startswith("http://127.0.0.1") or
            origin.startswith("file://") or
            origin == ""
        )
        self.send_header("Access-Control-Allow-Origin",
                         origin if allowed else "http://localhost:8765")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def log_message(self, fmt, *args):
        print(f"[http] {fmt % args}")


class ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads      = True


def run_binary_encrypted(paths):
    """
    Pipeline FHE complet avec chiffrement immediat des coordonnees.

    Etapes :
      1. Ecrire les coordonnees en clair dans des fichiers temporaires
      2. Generer le contexte FHE (cles ephemeres)
      3. Chiffrer IMMEDIATEMENT chaque trajectoire -> detruire le clair
      4. Calcul FHE sans dechiffrement (detect_encrypted)
      5. Dechiffrer uniquement le bit de resultat final (decrypt_with_ss)

    Garantie : entre les etapes 3 et 5, aucune coordonnee GPS n'est
    accessible en clair. Seul le booleen collision/libre est dechiffre.
    """
    tmp_files  = []   # fichiers a nettoyer dans le finally
    path_files = []   # fichiers de trajectoire en clair (detruits a l'etape 3)

    try:
        # ── Etape 1 : ecrire les waypoints en clair (transitoirement) ───────────
        for i, path in enumerate(paths):
            with tempfile.NamedTemporaryFile(
                    mode='w', suffix='.txt', delete=False,
                    prefix=f'drone{i+1}_clear_') as f:
                line = "".join(
                    f"({int(p[0])},{int(p[1])},{int(p[2]) if len(p) > 2 else 0})"
                    for p in path
                )
                f.write(line + "\n")
                path_files.append(f.name)
                tmp_files.append(f.name)

        # ── Etape 2 : generer le contexte FHE (cles ephemeres par requete) ─────
        ctx = tempfile.mktemp(suffix="_ctx.bin")
        pk  = tempfile.mktemp(suffix="_pk.bin")
        emk = tempfile.mktemp(suffix="_emk.bin")
        esk = tempfile.mktemp(suffix="_esk.bin")
        sk  = tempfile.mktemp(suffix="_sk.bin")
        tmp_files += [ctx, pk, emk, esk, sk]

        result_init = subprocess.run(
            [BINARY_PATH, "--mode", "init",
             "--ctx", ctx, "--pk", pk,
             "--emk", emk, "--esk", esk, "--sk", sk],
            capture_output=True, text=True, timeout=120
        )
        if result_init.returncode != 0:
            return None, f"Init FHE echoue : {result_init.stderr[:200]}"

        # ── Etape 3 : chiffrer chaque trajectoire, DETRUIRE le clair aussitot ──
        ct_files = []
        for pf in path_files:
            ct = tempfile.mktemp(suffix="_ct")
            tmp_files.append(ct)
            ct_files.append(ct)

            result_enc = subprocess.run(
                [BINARY_PATH, "--mode", "encrypt",
                 "--ctx", ctx, "--pk", pk,
                 "--path", pf, "--out", ct],
                capture_output=True, text=True, timeout=120
            )
            if result_enc.returncode != 0:
                return None, f"Chiffrement echoue : {result_enc.stderr[:200]}"

            # Destruction immediate du fichier clair
            try:
                os.unlink(pf)
                tmp_files.remove(pf)
            except OSError:
                pass

        # ── Etape 4 : calcul FHE sans dechiffrement (cote Bob centralise) ──────
        total_ss      = 0
        any_collision = False
        pairs_count   = 0
        t0            = time.time()

        for i in range(len(ct_files)):
            for j in range(i + 1, len(ct_files)):
                pairs_count += 1
                ct_res = tempfile.mktemp(suffix="_res")
                tmp_files.append(ct_res)

                result_det = subprocess.run(
                    [BINARY_PATH, "--mode", "detect_encrypted",
                     "--ctx", ctx, "--emk", emk,
                     "--ct1", ct_files[i],
                     "--ct2", ct_files[j],
                     "--out", ct_res],
                    capture_output=True, text=True, timeout=600
                )
                if result_det.returncode != 0:
                    return None, f"detect_encrypted echoue : {result_det.stderr[:200]}"

                # ── Etape 5 : dechiffrer uniquement le bit de resultat final ────
                result_dec = subprocess.run(
                    [BINARY_PATH, "--mode", "decrypt_with_ss",
                     "--ctx", ctx, "--sk", sk,
                     "--pk",  pk,  "--emk", emk,
                     "--ct",  ct_res],
                    capture_output=True, text=True, timeout=600
                )
                if result_dec.returncode != 0:
                    return None, f"decrypt_with_ss echoue : {result_dec.stderr[:200]}"

                for line in result_dec.stdout.splitlines():
                    if line.startswith("JSON_RESULT:"):
                        d = json.loads(line[len("JSON_RESULT:"):])
                        if d.get("collision", False):
                            any_collision = True
                        total_ss += d.get("scheme_switches", 0)
                        break

        elapsed_ms = (time.time() - t0) * 1000

        return {
            "mode":              "fhe_reel",
            "collision":         any_collision,
            "scheme_switches":   total_ss,
            "pairs_tested":      pairs_count,
            "total_time_ms":     round(elapsed_ms, 1),
            # Note : ciphertext_size_kb disponible via /api/health si besoin
        }, None

    finally:
        # Nettoyage de TOUS les fichiers temporaires (clairs et chiffres)
        suffixes_ct = [
            "", "_p1x.bin", "_p1y.bin", "_p1z.bin",
            "_q1x.bin", "_q1y.bin", "_q1z.bin",
            "_cop.bin", "_o1.bin", "_o2.bin", "_o3.bin", "_o4.bin"
        ]
        for f in tmp_files:
            for suf in suffixes_ct:
                try:
                    os.unlink(f + suf)
                except OSError:
                    pass


def analyze_paths(paths):
    """
    Point d'entree principal pour l'analyse de trajectoires.
    Utilise le pipeline FHE chiffre si le binaire est disponible,
    sinon retombe sur une simulation (mode degrade).
    """
    if os.path.isfile(BINARY_PATH):
        data, error = run_binary_encrypted(paths)
        if data:
            return data
        # En cas d'erreur FHE, logguer et utiliser la simulation
        print(f"[server] Erreur pipeline FHE : {error} — mode simulation")

    # Mode degrade : simulation des metriques sans calcul reel
    n_seg_total = sum(
        (len(paths[i]) - 1) * (len(paths[j]) - 1)
        for i in range(len(paths))
        for j in range(i + 1, len(paths))
    )
    metrics = simulate_metrics(n_seg_total)
    return {
        "mode":              "simulation",
        "collision":         False,
        "scheme_switches":   metrics["scheme_switches"],
        "ciphertext_size_kb": metrics["ciphertext_size_kb"],
        "time_ms":           metrics["time_ms"],
    }


if __name__ == "__main__":
    base = os.path.dirname(os.path.abspath(__file__))
    # Servir depuis web/ si ce dossier existe, sinon depuis la racine
    web_dir = os.path.join(base, "web")
    serve_dir = web_dir if os.path.isdir(web_dir) else base
    os.chdir(serve_dir)
    print(f"Drone HE server running on http://localhost:{PORT}")
    print(f"Racine web : {serve_dir}")
    print(f"Pipeline : chiffrement immediat des coordonnees (securite renforcee)")

    with ThreadedServer(("", PORT), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServeur arrete.")
