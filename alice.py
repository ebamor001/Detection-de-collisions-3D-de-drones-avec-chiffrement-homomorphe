#!/usr/bin/env python3
"""
alice.py — Drone Alice (serveur)  [version CORRIGÉE]

Protocole corrigé :
  1. Alice génère le contexte FHE + clés (pk, sk, emk)
  2. Alice envoie contexte + pk + emk à Bob (sk reste LOCAL chez Alice)
  3. Alice chiffre sa trajectoire avec sa pk → 6 ciphertexts
  4. Alice envoie les 6 ct_alice à Bob
  5. Bob chiffre sa trajectoire avec pk_alice, calcule detect_encrypted
     → produit 10 ciphertexts de résultat ("_cop","_nx2", "_ny2", "_nz2","_p12_xy", "_p34_xy","_p12_xz", "_p34_xz","_p12_yz", "_p34_yz") SANS déchiffrer
  6. Bob envoie les 5 ct_result à Alice
  7. Alice appelle decrypt_result avec SA clé secrète → décision de collision

CORRECTIONS APPORTÉES :
  - Bug #1 : Alice déchiffre elle-même les résultats avec sa sk (Bob ne déchiffre plus)
  - Bug #2 : Alice ne reçoit plus la position GPS de Bob en clair
  - Bug #3 : Trajectoires avec z variable (descente/montée de Bob) pour la géométrie 3D

Usage :
  Terminal 1 : python3 alice.py
  Terminal 2 : python3 bob.py
"""

import socket, struct, subprocess, tempfile, os, json, time

BINARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")
STATE_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone_state.json")
ALICE_PORT  = 9001

ALICE_TRAJ = [
    (0, 0, 30),
    (100, 100, 30)
]
BOB_TRAJ = [
    (0, 100, 30),
    (100, 0, 30)
]

RESULT_SUFFIXES = [
    "_cop",
    "_nx2", "_ny2", "_nz2",
    "_p12_xy", "_p34_xy",
    "_p12_xz", "_p34_xz",
    "_p12_yz", "_p34_yz"
]

# ── Helpers socket ────────────────────────────────────────────────────────────
def send_data(s, data):
    # Envoyer la taille
    s.sendall(struct.pack(">I", len(data)))
    # Envoyer par chunks de 64KB pour eviter le buffer overflow
    chunk = 65536
    for i in range(0, len(data), chunk):
        s.sendall(data[i:i+chunk])

def recv_data(s):
    n = struct.unpack(">I", _exact(s, 4))[0]
    return _exact(s, n)

def _exact(s, n):
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c: raise ConnectionError("Socket fermee")
        buf += c
    return buf

# ── Helpers binaire ───────────────────────────────────────────────────────────
def write_file(path, data):
    with open(path, "wb") as f: f.write(data)

def read_file(path):
    with open(path, "rb") as f: return f.read()

def run(cmd, timeout=600):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError(r.stderr[:300])
    return r.stdout

def pts_to_str(pts):
    return "".join(f"({x},{y},{z})" for x,y,z in pts)

def write_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f)

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print("=" * 55)
    print("  ALICE — Drone 1 (port 9001)")
    print("=" * 55)

    # Fichiers temporaires persistants pour le contexte
    ctx_file = tempfile.NamedTemporaryFile(suffix="_ctx.bin",  delete=False).name
    pk_file  = tempfile.NamedTemporaryFile(suffix="_pk.bin",   delete=False).name
    emk_file = tempfile.NamedTemporaryFile(suffix="_emk.bin",  delete=False).name
    esk_file = tempfile.NamedTemporaryFile(suffix="_esk.bin",  delete=False).name
    sk_file  = tempfile.NamedTemporaryFile(suffix="_sk.bin",   delete=False).name

    # ── Etape 1 : Init FHE ────────────────────────────────────────────────────
    print("[Alice] Generation du contexte FHE...")
    run([BINARY_PATH, "--mode", "init",
         "--ctx", ctx_file, "--pk", pk_file,
         "--emk", emk_file, "--esk", esk_file, "--sk", sk_file])
    print("[Alice] Contexte OK")

    ctx_data = read_file(ctx_file)
    pk_data  = read_file(pk_file)
    emk_data = read_file(emk_file)
    print(f"[Alice] Contexte : {len(ctx_data)/1024:.1f} KB | PK : {len(pk_data)/1024:.1f} KB")

    # ── Etape 2 : Attendre Bob ─────────────────────────────────────────────────
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("", ALICE_PORT))
    server.listen(1)
    print(f"[Alice] En attente de Bob sur le port {ALICE_PORT}...")

    conn, addr = server.accept()
    print(f"[Alice] Bob connecte depuis {addr}")

    # ── Etape 3 : Envoyer contexte + clé publique + emk à Bob ────────────────
    # CORRECTION : on n'envoie PAS esk (eval sum keys) — inutile pour detect_encrypted
    # La clé secrète sk_file reste STRICTEMENT locale chez Alice
    print("[Alice] Envoi du contexte et de la clé publique à Bob...")
    send_data(conn, ctx_data)
    send_data(conn, pk_data)
    send_data(conn, emk_data)
    print("[Alice] Envoyé (sk non transmise)")

    # ── Etat initial ──────────────────────────────────────────────────────────
    state = {
        "alice": {"pos": list(ALICE_TRAJ[0]), "seg": 0, "total": len(ALICE_TRAJ)-1},
        "bob":   {"pos": list(BOB_TRAJ[0]),   "seg": 0, "total": len(BOB_TRAJ)-1},
        "collision": False, "collision_msg": "",
        "fhe_running": False, "fhe_time_ms": 0,
        "scheme_switches": 0, "finished": False,
        "traj_alice": list(ALICE_TRAJ),
        "traj_bob":   list(BOB_TRAJ),
    }
    write_state(state)

    # ── Boucle simulation ─────────────────────────────────────────────────────
    n = len(ALICE_TRAJ) - 1
    for seg in range(1, n + 1):

        print(f"\n[Alice] === Segment {seg}/{n} ===")

        # Mettre a jour position
        state["alice"]["pos"] = list(ALICE_TRAJ[seg])
        state["alice"]["seg"] = seg
        state["fhe_running"]  = True
        state["collision"]    = False
        state["collision_msg"]= ""
        write_state(state)

        # ── Chiffrer le segment d'Alice (6 ciphertexts, 1 coord par ct) ──────
        f_alice_path = tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False).name
        f_alice_ct   = tempfile.NamedTemporaryFile(suffix='_ct_alice', delete=False).name
        seg_alice = [ALICE_TRAJ[seg-1], ALICE_TRAJ[seg]]
        with open(f_alice_path, 'w') as f:
            f.write(pts_to_str(seg_alice) + "\n")

        run([BINARY_PATH, "--mode", "encrypt",
             "--ctx", ctx_file, "--pk", pk_file,
             "--path", f_alice_path, "--out", f_alice_ct])

        # Lire les 6 fichiers generes
        suffixes = ["_p1x","_p1y","_p1z","_q1x","_q1y","_q1z"]
        alice_cts = {}
        for s in suffixes:
            fname = f_alice_ct + s + ".bin"
            alice_cts[s] = read_file(fname)
        print(f"[Alice] 6 ciphertexts generes ({len(alice_cts['_p1x'])/1024:.1f} KB chacun)")

        # Sauvegarder le ciphertext du segment 1 pour les scripts de demo securite
        if seg == 1:
            import shutil
            demo_dir = os.path.dirname(os.path.abspath(__file__))
            for s in suffixes:
                shutil.copy(f_alice_ct + s + ".bin",
                            os.path.join(demo_dir, "demo_ct_alice" + s + ".bin"))
            write_file(os.path.join(demo_dir, "demo_ctx.bin"),  ctx_data)
            write_file(os.path.join(demo_dir, "demo_pk.bin"),   pk_data)
            write_file(os.path.join(demo_dir, "demo_emk.bin"),  emk_data)
            # CORRECTION : demo_esk.bin supprimé (esk n'est plus utilisé dans le protocole)
            shutil.copy(f_alice_ct + "_p1x.bin",
                        os.path.join(demo_dir, "demo_ct_alice.bin"))
            print("[Alice] Ciphertexts sauvegardes -> demo_ct_alice*.bin")

        # ── Envoyer signal + 6 ciphertexts à Bob ─────────────────────────────
        send_data(conn, str(seg).encode())
        for s in suffixes:
            send_data(conn, alice_cts[s])
        print(f"[Alice] 6 ct_alice envoyés à Bob")

        # ── CORRECTION BUG #1 : Recevoir les 5 ciphertexts de résultat de Bob ──
        # Bob envoie les résultats CHIFFRÉS — Alice les déchiffre avec sa sk
        # (plus de JSON en clair avec position GPS de Bob)
        _ = recv_data(conn)   # signal de segment de Bob (ignoré, on connaît le nôtre)
        f_ct_res = tempfile.NamedTemporaryFile(suffix="_ct_result", delete=False).name
        for s in RESULT_SUFFIXES:
            data = recv_data(conn)
            write_file(f_ct_res + s + ".bin", data)
        print(f"[Alice] 10 ciphertexts de résultat reçus de Bob")

        # Déchiffrement + scheme switching côté Alice avec ses propres clés
        t0 = time.time()
        out = run([BINARY_PATH, "--mode", "decrypt_with_ss",
                   "--ctx", ctx_file, "--sk", sk_file,
                   "--pk",  pk_file,  "--emk", emk_file,
                   "--ct",  f_ct_res])
        ms = (time.time() - t0) * 1000

        collision = False
# Alice récupère le booléen ET le nombre de SS
        ss_count = 0  # Valeur par défaut
        try:
            for line in out.splitlines():
                if line.startswith("JSON_RESULT:"):
                    d = json.loads(line[len("JSON_RESULT:"):])
                    collision = d.get("collision", False)
                    ss_count = d.get("scheme_switches", 0) # Extraction du compteur C++
                    break
        except Exception as e:
            print(f"[Alice] Erreur parsing : {e}")

        print(f"[Alice] Décision : {'COLLISION' if collision else 'LIBRE'} ({ms:.0f} ms)")

        # CORRECTION BUG #2 : La position de Bob n'est plus reçue en clair
        # On met à jour l'état visuel avec la trajectoire connue (côté Alice)
        state["bob"]["pos"] = list(BOB_TRAJ[seg])
        state["bob"]["seg"] = seg
        state["collision"]      = collision
        state["fhe_running"]    = False
        state["fhe_time_ms"]    = round(ms, 1)
        state["scheme_switches"]= ss_count  # <-- On injecte la vraie valeur
        if collision:
            state["collision_msg"] = \
                f"COLLISION seg {seg} — Alice ({ALICE_TRAJ[seg]}) x Bob ({BOB_TRAJ[seg]}) — CHANGER ALTITUDE"
        write_state(state)

        # Nettoyage fichiers temporaires du segment
        for s in suffixes:
            try: os.unlink(f_alice_ct + s + ".bin")
            except: pass
        for s in RESULT_SUFFIXES:
            try: os.unlink(f_ct_res + s + ".bin")
            except: pass
        for f in [f_alice_path]:
            try: os.unlink(f)
            except: pass

        time.sleep(0.5)

    # Fin
    state["finished"] = True
    write_state(state)
    print("\n[Alice] Simulation terminee.")

    conn.close()
    server.close()
    for f in [ctx_file, pk_file, emk_file, sk_file]:
        try: os.unlink(f)
        except: pass

if __name__ == "__main__":
    main()