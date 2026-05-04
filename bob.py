#!/usr/bin/env python3
"""
bob.py — Drone Bob  (version CORRIGÉE)

Protocole corrigé :
  1. Alice génère pk + sk
  2. Alice chiffre sa trajectoire avec pk_alice → 6 ciphertexts
  3. Alice envoie les 6 ct_alice + ctx + emk à Bob
  4. Bob chiffre sa trajectoire avec pk_alice → 6 ciphertexts
  5. Bob appelle --mode detect_encrypted
     Calcul FHE complet avec le contexte d'Alice — JAMAIS de déchiffrement côté Bob
  6. Bob envoie les 22 ciphertexts de résultat ("_cop","_nx2", "_ny2", "_nz2","_p12_xy", "_p34_xy","_p12_xz", "_p34_xz","_p12_yz", "_p34_yz") à Alice
  7. Alice déchiffre avec SA clé secrète et prend la décision

CORRECTIONS APPORTÉES :
  - Bug #1 : Bob n'appelle plus engine.decryptValue() avec un engine aux mauvaises clés.
             Il envoie les ciphertexts chiffrés à Alice pour déchiffrement.
  - Bug #2 : Bob ne révèle plus sa position GPS en clair dans le réseau.
             (suppression de "pos" et "seg" du JSON envoyé à Alice)
  - Bug #3 : Trajectoires avec altitude z variable pour tester la géométrie 3D.
"""

import socket, struct, subprocess, tempfile, os, json, time


MAX_REQUESTS = 5


BINARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")
ALICE_HOST  = "127.0.0.1"
ALICE_PORT  = 9001

BOB_TRAJ = [
    (0, 100, 30),
    (100, 0, 30)
]

SUFFIXES = ["_p1x","_p1y","_p1z","_q1x","_q1y","_q1z"]

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

def main():
    print("=" * 55)
    print("  BOB — Drone 2 (calcul FHE sur ciphertexts)")
    print("=" * 55)

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

    # Recevoir contexte + clés d'Alice (CORRECTION : plus d'esk)
    ctx_data = recv_data(sock)
    pk_data  = recv_data(sock)
    emk_data = recv_data(sock)
    print(f"[Bob] Contexte : {len(ctx_data)/1024:.1f} KB | PK Alice : {len(pk_data)/1024:.1f} KB")

    ctx_file = tempfile.NamedTemporaryFile(suffix="_ctx.bin", delete=False).name
    pk_file  = tempfile.NamedTemporaryFile(suffix="_pk.bin",  delete=False).name
    emk_file = tempfile.NamedTemporaryFile(suffix="_emk.bin", delete=False).name

    write_file(ctx_file, ctx_data)
    write_file(pk_file,  pk_data)
    write_file(emk_file, emk_data)

    n = len(BOB_TRAJ) - 1
    request_count = 0
    last_seg = 0

    for seg in range(1, n + 1):

        # Signal d'Alice + reception des 6 ciphertexts d'Alice
        seg_signal = recv_data(sock).decode()
        print(f"\n[Bob] === Segment {seg_signal} signal par Alice ===")

        request_count += 1

        if request_count > MAX_REQUESTS:
            raise RuntimeError("[SECURITY] Trop de requêtes Alice — session bloquée")

        seg_recv = int(seg_signal)
        if seg_recv != last_seg + 1:
            raise RuntimeError("[SECURITY] Requête incohérente — ordre des segments invalide")

        last_seg = seg_recv

        # Recevoir les 6 ct_alice
        f_ct_alice = tempfile.NamedTemporaryFile(suffix="_ct_alice", delete=False).name
        for s in SUFFIXES:
            data = recv_data(sock)
            write_file(f_ct_alice + s + ".bin", data)
        print(f"[Bob] 6 ct_alice recus")

        # Chiffrer le segment de Bob avec pk_alice (6 ciphertexts)
        seg_bob = [BOB_TRAJ[seg-1], BOB_TRAJ[seg]]
        print(f"[Bob] Segment : {seg_bob[0]} -> {seg_bob[1]}")

        f_bob_path = tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False).name
        f_ct_bob   = tempfile.NamedTemporaryFile(suffix="_ct_bob", delete=False).name
        # CORRECTION BUG #1 : préfixe pour les 22ciphertexts de résultat
        f_ct_res   = tempfile.NamedTemporaryFile(suffix="_ct_result", delete=False).name

        with open(f_bob_path, 'w') as f:
            f.write(pts_to_str(seg_bob) + "\n")

        run([BINARY_PATH, "--mode", "encrypt",
             "--ctx", ctx_file, "--pk", pk_file,
             "--path", f_bob_path, "--out", f_ct_bob])
        print(f"[Bob] 6 ct_bob générés avec pk_alice")

        # CORRECTION BUG #1 : Calcul FHE sans déchiffrement côté Bob
        # detect_encrypted écrit 22 ciphertexts résultat (cop, o1..o4)
        # que seule Alice peut déchiffrer avec sa clé secrète
        print("[Bob] Calcul FHE detect_encrypted (sans déchiffrement)...")
        t0 = time.time()
        run([BINARY_PATH, "--mode", "detect_encrypted",
             "--ctx", ctx_file,
             "--emk", emk_file,
             "--ct1", f_ct_alice,
             "--ct2", f_ct_bob,
             "--out", f_ct_res])   # ← 5 fichiers ct_result_*.bin générés
        ms = (time.time() - t0) * 1000
        print(f"[Bob] Calcul terminé ({ms:.0f} ms) — ciphertexts prêts pour Alice")

        # Lire les 10 ciphertexts de résultat
        RESULT_SUFFIXES = [
            "_cop",
            "_nx2", "_ny2", "_nz2",

            "_p12_xy", "_p34_xy",
            "_p12_xz", "_p34_xz",
            "_p12_yz", "_p34_yz",

            "_o1_xy", "_o2_xy", "_o3_xy", "_o4_xy",
            "_o1_xz", "_o2_xz", "_o3_xz", "_o4_xz",
            "_o1_yz", "_o2_yz", "_o3_yz", "_o4_yz"
        ]
        result_cts = {}
        for s in RESULT_SUFFIXES:
            result_cts[s] = read_file(f_ct_res + s + ".bin")
        print(f"[Bob] 22 ciphertexts de résultat lus ({len(result_cts['_cop'])/1024:.1f} KB chacun)")

        # CORRECTION BUG #2 : Bob envoie les ciphertexts chiffrés, JAMAIS sa position GPS
        # La position de Bob reste confidentielle — Alice déchiffrera avec sa clé secrète
        send_data(sock, str(seg).encode())           # signal de segment
        for s in RESULT_SUFFIXES:
            send_data(sock, result_cts[s])           # 10 ciphertexts (≈ 10× taille d'un ct)
        print(f"[Bob] 22 ct_result envoyés à Alice (position Bob non révélée)")

        # Nettoyage
        for s in SUFFIXES:
            for pref in [f_ct_alice, f_ct_bob]:
                try: os.unlink(pref + s + ".bin")
                except: pass
        for s in RESULT_SUFFIXES:
            try: os.unlink(f_ct_res + s + ".bin")
            except: pass
        try: os.unlink(f_bob_path)
        except: pass

    print("\n[Bob] Simulation terminée.")
    sock.close()
    for f in [ctx_file, pk_file, emk_file]:
        try: os.unlink(f)
        except: pass

if __name__ == "__main__":
    main()