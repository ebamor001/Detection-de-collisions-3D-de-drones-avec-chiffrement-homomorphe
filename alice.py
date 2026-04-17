#!/usr/bin/env python3
"""
alice.py — Drone Alice (serveur)
- Genere le contexte FHE + cles
- Envoie contexte + cle publique + eval keys a Bob via socket
- Recoit les ciphertexts de Bob (chiffres avec la cle d'Alice)
- Chiffre sa propre trajectoire avec sa cle publique
- Fait le calcul FHE sur les deux ciphertexts
- Ecrit drone_state.json pour live.html
- Repete pour chaque segment (simulation temps reel)

Usage :
  Terminal 1 : python3 alice.py
  Terminal 2 : python3 bob.py
  Navigateur : http://localhost:8765/web/live.html
"""

import socket, struct, subprocess, tempfile, os, json, time, threading

BINARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")
STATE_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone_state.json")
ALICE_PORT  = 9001

# Trajectoire Alice : 16 points = 15 segments, croise Bob vers le milieu
ALICE_TRAJ = [
    (0,50,30),(6,54,30),(13,58,30),(20,61,30),(26,64,30),
    (33,67,30),(40,69,30),(46,69,30),(53,69,30),(60,69,30),
    (66,67,30),(73,64,30),(80,61,30),(86,58,30),(93,54,30),(100,50,30)
]
BOB_TRAJ = [
    (100,0,30),(98,6,30),(95,13,30),(90,20,30),(83,26,30),
    (75,33,30),(65,40,30),(55,46,30),(44,53,30),(34,60,30),
    (25,66,30),(16,73,30),(9,80,30),(4,86,30),(1,93,30),(0,100,30)
]

# ── Helpers socket ────────────────────────────────────────────────────────────
def send_data(s, data):
    s.sendall(struct.pack(">I", len(data)) + data)

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
    esk_data = read_file(esk_file)
    print(f"[Alice] Contexte : {len(ctx_data)/1024:.1f} KB | PK : {len(pk_data)/1024:.1f} KB")

    # ── Etape 2 : Attendre Bob ─────────────────────────────────────────────────
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("", ALICE_PORT))
    server.listen(1)
    print(f"[Alice] En attente de Bob sur le port {ALICE_PORT}...")

    conn, addr = server.accept()
    print(f"[Alice] Bob connecte depuis {addr}")

    # ── Etape 3 : Envoyer contexte + cle publique a Bob ───────────────────────
    print("[Alice] Envoi du contexte et de la cle publique a Bob...")
    send_data(conn, ctx_data)
    send_data(conn, pk_data)
    send_data(conn, emk_data)
    send_data(conn, esk_data)
    print("[Alice] Envoye")

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

        # Segment courant d'Alice
        seg_alice = [ALICE_TRAJ[seg-1], ALICE_TRAJ[seg]]

        # ── Chiffrer le segment d'Alice avec SA cle publique ──────────────────
        f_alice_path = tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False).name
        f_alice_ct   = tempfile.NamedTemporaryFile(suffix='_ct_alice.bin', delete=False).name
        with open(f_alice_path, 'w') as f:
            f.write(pts_to_str(seg_alice) + "\n")

        run([BINARY_PATH, "--mode", "encrypt",
             "--ctx", ctx_file, "--pk", pk_file,
             "--path", f_alice_path, "--out", f_alice_ct])

        alice_ct_data = read_file(f_alice_ct)
        print(f"[Alice] Ciphertext Alice : {len(alice_ct_data)/1024:.1f} KB")

        # ── Envoyer le numero de segment a Bob ────────────────────────────────
        send_data(conn, str(seg).encode())

        # ── Recevoir le ciphertext de Bob (chiffre avec la cle d'Alice) ───────
        bob_ct_data = recv_data(conn)
        f_bob_ct = tempfile.NamedTemporaryFile(suffix='_ct_bob.bin', delete=False).name
        write_file(f_bob_ct, bob_ct_data)
        print(f"[Alice] Ciphertext Bob recu : {len(bob_ct_data)/1024:.1f} KB")

        # Mettre a jour position de Bob (envoyee par Bob)
        bob_pos_data = recv_data(conn)
        bob_pos = json.loads(bob_pos_data.decode())
        state["bob"]["pos"] = bob_pos["pos"]
        state["bob"]["seg"] = bob_pos["seg"]

        # ── Detection FHE ─────────────────────────────────────────────────────
        print("[Alice] Calcul FHE...")
        t0 = time.time()
        out = run([BINARY_PATH, "--mode", "detect",
                   "--ctx", ctx_file, "--emk", emk_file, "--esk", esk_file,
                   "--sk",  sk_file,
                   "--ct1", f_alice_ct, "--ct2", f_bob_ct])
        ms = (time.time() - t0) * 1000

        # Parser JSON_RESULT
        collision = False
        ss = 0
        for line in out.splitlines():
            if line.startswith("JSON_RESULT:"):
                d = json.loads(line[len("JSON_RESULT:"):])
                collision = d.get("collision", False)
                ss        = d.get("scheme_switches", 0)
                break

        print(f"[Alice] Resultat : {'COLLISION' if collision else 'LIBRE'} ({ms:.0f}ms, {ss} SS)")

        # Mettre a jour l'etat
        state["collision"]      = collision
        state["fhe_running"]    = False
        state["fhe_time_ms"]    = round(ms, 1)
        state["scheme_switches"]= ss
        if collision:
            state["collision_msg"] = \
                f"COLLISION seg {seg} — Alice ({ALICE_TRAJ[seg]}) x Bob ({BOB_TRAJ[seg]}) — CHANGER ALTITUDE"
        write_state(state)

        # Nettoyage fichiers temporaires du segment
        for f in [f_alice_path, f_alice_ct, f_bob_ct]:
            try: os.unlink(f)
            except: pass

        time.sleep(0.5)

    # Fin
    state["finished"] = True
    write_state(state)
    print("\n[Alice] Simulation terminee.")

    conn.close()
    server.close()
    for f in [ctx_file, pk_file, emk_file, esk_file, sk_file]:
        try: os.unlink(f)
        except: pass

if __name__ == "__main__":
    main()
