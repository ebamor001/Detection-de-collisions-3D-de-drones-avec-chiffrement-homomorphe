#!/usr/bin/env python3
"""
bob.py — Drone Bob (client)
- Se connecte a Alice via socket
- Recoit le contexte + cle publique d'Alice
- A chaque segment : chiffre sa trajectoire AVEC LA CLE D'ALICE
- Envoie le ciphertext a Alice
- Alice fait le calcul FHE et ecrit le resultat

Usage :
  Terminal 1 : python3 alice.py  (lancer en premier)
  Terminal 2 : python3 bob.py
"""

import socket, struct, subprocess, tempfile, os, json, time

BINARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")
ALICE_HOST  = "127.0.0.1"
ALICE_PORT  = 9001

# Meme trajectoire que dans alice.py
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

# ── Helpers ───────────────────────────────────────────────────────────────────
def write_file(path, data):
    with open(path, "wb") as f: f.write(data)

def run(cmd, timeout=600):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError(r.stderr[:300])
    return r.stdout

def pts_to_str(pts):
    return "".join(f"({x},{y},{z})" for x,y,z in pts)

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print("=" * 55)
    print("  BOB — Drone 2")
    print("=" * 55)

    # ── Connexion a Alice ─────────────────────────────────────────────────────
    print(f"[Bob] Connexion a Alice sur {ALICE_HOST}:{ALICE_PORT}...")
    sock = None
    for attempt in range(15):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ALICE_HOST, ALICE_PORT))
            break
        except ConnectionRefusedError:
            print(f"[Bob] Alice pas prete, attente... ({attempt+1}/15)")
            time.sleep(2)
            sock = None

    if sock is None:
        raise RuntimeError("Impossible de se connecter a Alice")
    print("[Bob] Connecte a Alice")

    # ── Recevoir contexte + cle publique d'Alice ──────────────────────────────
    print("[Bob] Reception du contexte d'Alice...")
    ctx_data = recv_data(sock)
    pk_data  = recv_data(sock)
    emk_data = recv_data(sock)
    esk_data = recv_data(sock)
    print(f"[Bob] Contexte : {len(ctx_data)/1024:.1f} KB | PK Alice : {len(pk_data)/1024:.1f} KB")

    # Sauvegarder dans des fichiers temporaires
    ctx_file = tempfile.NamedTemporaryFile(suffix="_ctx.bin", delete=False).name
    pk_file  = tempfile.NamedTemporaryFile(suffix="_pk.bin",  delete=False).name
    emk_file = tempfile.NamedTemporaryFile(suffix="_emk.bin", delete=False).name
    esk_file = tempfile.NamedTemporaryFile(suffix="_esk.bin", delete=False).name

    write_file(ctx_file, ctx_data)
    write_file(pk_file,  pk_data)   # CLE PUBLIQUE D'ALICE
    write_file(emk_file, emk_data)
    write_file(esk_file, esk_data)

    # ── Boucle simulation ─────────────────────────────────────────────────────
    n = len(BOB_TRAJ) - 1
    for seg in range(1, n + 1):

        # Attendre le signal d'Alice (numero de segment)
        seg_signal = recv_data(sock).decode()
        print(f"\n[Bob] === Segment {seg_signal} signal par Alice ===")

        # Segment courant de Bob
        seg_bob = [BOB_TRAJ[seg-1], BOB_TRAJ[seg]]
        print(f"[Bob] Segment : {seg_bob[0]} → {seg_bob[1]}")

        # ── Chiffrer avec LA CLE PUBLIQUE D'ALICE ────────────────────────────
        # Bob chiffre ses coordonnees avec la cle d'Alice
        # => seule Alice peut dechiffrer
        f_path = tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False).name
        f_ct   = tempfile.NamedTemporaryFile(suffix='_ct_bob.bin', delete=False).name

        with open(f_path, 'w') as f:
            f.write(pts_to_str(seg_bob) + "\n")

        print("[Bob] Chiffrement avec la cle publique d'Alice...")
        run([BINARY_PATH, "--mode", "encrypt",
             "--ctx",  ctx_file,
             "--pk",   pk_file,    # CLE D'ALICE
             "--path", f_path,
             "--out",  f_ct])

        ct_data = open(f_ct, "rb").read()
        print(f"[Bob] Ciphertext : {len(ct_data)/1024:.1f} KB — envoi a Alice...")

        # ── Envoyer le ciphertext a Alice ─────────────────────────────────────
        send_data(sock, ct_data)

        # ── Envoyer la position de Bob a Alice (pour la carte) ───────────────
        pos_info = json.dumps({"pos": list(BOB_TRAJ[seg]), "seg": seg}).encode()
        send_data(sock, pos_info)

        print(f"[Bob] Ciphertext envoye — en attente du prochain segment...")

        # Nettoyage
        for f in [f_path, f_ct]:
            try: os.unlink(f)
            except: pass

    print("\n[Bob] Simulation terminee.")
    sock.close()

    for f in [ctx_file, pk_file, emk_file, esk_file]:
        try: os.unlink(f)
        except: pass

if __name__ == "__main__":
    main()
