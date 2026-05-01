#!/usr/bin/env python3
"""
bob.py — Drone Bob
Protocole :
  1. Alice genere pk + sk
  2. Alice chiffre sa trajectoire avec pk_alice → 6 ciphertexts
  3. Alice envoie les 6 ct_alice + pk + ctx + emk + esk a Bob
  4. Bob chiffre sa trajectoire avec pk_alice → 6 ciphertexts
  5. Bob appelle --mode detect_encrypted avec 12 ciphertexts
     Calcul FHE complet sans dechiffrer les coordonnees
  6. Bob envoie UNIQUEMENT le resultat JSON a Alice
  7. Alice prend la decision
"""

import socket, struct, subprocess, tempfile, os, json, time

BINARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")
ALICE_HOST  = "127.0.0.1"
ALICE_PORT  = 9001

BOB_TRAJ = [
    (100,0,30),(98,6,30),(95,13,30),(90,20,30),(83,26,30),
    (75,33,30),(65,40,30),(55,46,30),(44,53,30),(34,60,30),
    (25,66,30),(16,73,30),(9,80,30),(4,86,30),(1,93,30),(0,100,30)
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

    # Recevoir contexte + cles d'Alice
    ctx_data = recv_data(sock)
    pk_data  = recv_data(sock)
    emk_data = recv_data(sock)
    esk_data = recv_data(sock)
    print(f"[Bob] Contexte : {len(ctx_data)/1024:.1f} KB | PK Alice : {len(pk_data)/1024:.1f} KB")

    ctx_file = tempfile.NamedTemporaryFile(suffix="_ctx.bin", delete=False).name
    pk_file  = tempfile.NamedTemporaryFile(suffix="_pk.bin",  delete=False).name
    emk_file = tempfile.NamedTemporaryFile(suffix="_emk.bin", delete=False).name
    esk_file = tempfile.NamedTemporaryFile(suffix="_esk.bin", delete=False).name

    write_file(ctx_file, ctx_data)
    write_file(pk_file,  pk_data)
    write_file(emk_file, emk_data)
    write_file(esk_file, esk_data)

    n = len(BOB_TRAJ) - 1
    for seg in range(1, n + 1):

        # Signal d'Alice + reception des 6 ciphertexts d'Alice
        seg_signal = recv_data(sock).decode()
        print(f"\n[Bob] === Segment {seg_signal} signal par Alice ===")

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

        with open(f_bob_path, 'w') as f:
            f.write(pts_to_str(seg_bob) + "\n")

        run([BINARY_PATH, "--mode", "encrypt",
             "--ctx", ctx_file, "--pk", pk_file,
             "--path", f_bob_path, "--out", f_ct_bob])
        print(f"[Bob] 6 ct_bob generes avec pk_alice")

        # Calcul FHE sur 12 ciphertexts — jamais de dechiffrement des coords
        print("[Bob] Calcul FHE detect_encrypted...")
        t0 = time.time()
        out = run([BINARY_PATH, "--mode", "detect_encrypted",
                   "--ctx", ctx_file,
                   "--emk", emk_file,
                   "--esk", esk_file,
                   "--ct1", f_ct_alice,
                   "--ct2", f_ct_bob])
        ms = (time.time() - t0) * 1000

        collision = False
        ss = 0
        try:
            for line in out.splitlines():
                if line.startswith("JSON_RESULT:"):
                    d = json.loads(line[len("JSON_RESULT:"):])
                    collision = d.get("collision", False)
                    ss        = d.get("scheme_switches", 0)
                    break
        except Exception as e:
            print(f"[Bob] Erreur parsing : {e}")

        print(f"[Bob] Resultat : {'COLLISION' if collision else 'LIBRE'} ({ms:.0f}ms, {ss} SS)")

        # Envoyer UNIQUEMENT le resultat — jamais les coords de Bob
        result_data = json.dumps({
            "collision":       collision,
            "scheme_switches": ss,
            "time_ms":         round(ms, 1),
            "pos":             list(BOB_TRAJ[seg]),
            "seg":             seg
        }).encode()
        send_data(sock, result_data)
        print(f"[Bob] Resultat envoye (coords Bob non divulguees)")

        # Nettoyage
        for s in SUFFIXES:
            for pref in [f_ct_alice, f_ct_bob]:
                try: os.unlink(pref + s + ".bin")
                except: pass
        try: os.unlink(f_bob_path)
        except: pass

    print("\n[Bob] Simulation terminee.")
    sock.close()
    for f in [ctx_file, pk_file, emk_file, esk_file]:
        try: os.unlink(f)
        except: pass

if __name__ == "__main__":
    main()
