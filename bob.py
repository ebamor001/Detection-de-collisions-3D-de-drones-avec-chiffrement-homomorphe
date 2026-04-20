#!/usr/bin/env python3
"""
bob.py — Drone Bob (client)

Nouveau protocole propre :
  1. Bob genere toute la session HE
  2. Bob se connecte a Alice
  3. Bob envoie ctx + pk  + emk + esk
  4. Pour chaque segment :
       - recoit le numero de segment d'Alice
       - chiffre son segment avec pk_bob
       - envoie ct_bob + position
       - recoit le resultat chiffre
       - dechiffre localement avec sk_bob
"""

import socket
import struct
import subprocess
import tempfile
import os
import json
import time

BINARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")
ALICE_HOST = "127.0.0.1"
ALICE_PORT = 9001

BOB_TRAJ = [
    (100, 0, 30), (98, 6, 30), (95, 13, 30), (90, 20, 30), (83, 26, 30),
    (75, 33, 30), (65, 40, 30), (55, 46, 30), (44, 53, 30), (34, 60, 30),
    (25, 66, 30), (16, 73, 30), (9, 80, 30), (4, 86, 30), (1, 93, 30), (0, 100, 30)
]

def send_data(sock, data):
    sock.sendall(struct.pack(">I", len(data)) + data)

def recv_data(sock):
    n = struct.unpack(">I", _exact(sock, 4))[0]
    return _exact(sock, n)

def _exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket fermee")
        buf += chunk
    return buf

def write_file(path, data):
    with open(path, "wb") as f:
        f.write(data)

def read_file(path):
    with open(path, "rb") as f:
        return f.read()

def run(cmd, timeout=600):
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if result.returncode != 0:
        print("\n=== CMD ===")
        print(" ".join(cmd))
        print("\n=== STDOUT ===")
        print(result.stdout)
        print("\n=== STDERR ===")
        print(result.stderr)
        raise RuntimeError("Commande echouee")
    return result.stdout

def pts_to_str(pts):
    return "".join(f"({x},{y},{z})" for x, y, z in pts)

def main():
    print("=" * 55)
    print("  BOB — Drone 2 (client)")
    print("=" * 55)

    ctx_file = tempfile.NamedTemporaryFile(suffix="_ctx.bin", delete=False).name
    pk_bob_file = tempfile.NamedTemporaryFile(suffix="_pk_bob.bin", delete=False).name
    sk_bob_file = tempfile.NamedTemporaryFile(suffix="_sk_bob.bin", delete=False).name
    emk_bob_file = tempfile.NamedTemporaryFile(suffix="_emk_bob.bin", delete=False).name
    esk_bob_file = tempfile.NamedTemporaryFile(suffix="_esk_bob.bin", delete=False).name
    btk_bob_file = tempfile.NamedTemporaryFile(suffix="_btk_bob.bin", delete=False).name
    swkfc_bob_file = tempfile.NamedTemporaryFile(suffix="_swkfc_bob.bin", delete=False).name
    sock = None
    try:
        print("[Bob] Generation de la session HE complete...")
        run([
            BINARY_PATH, "--mode", "init",
            "--ctx", ctx_file,
            "--pk", pk_bob_file,
            "--sk", sk_bob_file,
            "--emk", emk_bob_file,
            "--esk", esk_bob_file,
            "--btk", btk_bob_file,
            "--swkfc", swkfc_bob_file

        ])
        print("[Bob] Session HE prete")

        print(f"[Bob] Connexion a Alice sur {ALICE_HOST}:{ALICE_PORT}...")
        for attempt in range(15):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((ALICE_HOST, ALICE_PORT))
                break
            except ConnectionRefusedError:
                print(f"[Bob] Alice pas prete, attente... ({attempt + 1}/15)")
                time.sleep(2)
                sock = None

        if sock is None:
            raise RuntimeError("Impossible de se connecter a Alice")

        print("[Bob] Connecte a Alice")

        print("[Bob] Envoi de ctx + pk_bob + emk_bob + esk_bob + btk_bob + swkfc_bob a Alice...")
        send_data(sock, read_file(ctx_file))
        send_data(sock, read_file(pk_bob_file))
        send_data(sock, read_file(emk_bob_file))
        send_data(sock, read_file(esk_bob_file))
        send_data(sock, read_file(btk_bob_file))
        send_data(sock, read_file(swkfc_bob_file))
        print("[Bob] Artefacts envoyes")

        n = len(BOB_TRAJ) - 1
        for seg in range(1, n + 1):
            seg_signal = recv_data(sock).decode()

            try:
                seg_recv = int(seg_signal)
            except ValueError:
                raise RuntimeError(f"Segment invalide recu depuis Alice: {seg_signal!r}")

            if seg_recv != seg:
                raise RuntimeError(f"Desync Alice/Bob : recu {seg_recv}, attendu {seg}")

            print(f"\n[Bob] === Segment {seg_recv} ===")

            seg_bob = [BOB_TRAJ[seg - 1], BOB_TRAJ[seg]]
            print(f"[Bob] Segment : {seg_bob[0]} -> {seg_bob[1]}")

            f_path = tempfile.NamedTemporaryFile(mode="w", suffix=".txt", delete=False).name
            ct_bob = tempfile.NamedTemporaryFile(suffix="_ct_bob.bin", delete=False).name
            result_ct_file = tempfile.NamedTemporaryFile(suffix="_ct_result.bin", delete=False).name

            try:
                with open(f_path, "w") as f:
                    f.write(pts_to_str(seg_bob) + "\n")

                print("[Bob] Chiffrement avec pk_bob...")
                run([
                    BINARY_PATH, "--mode", "encrypt",
                    "--ctx", ctx_file,
                    "--pk", pk_bob_file,
                    "--path", f_path,
                    "--out", ct_bob
                ])

                ct_bob_data = read_file(ct_bob)
                print(f"[Bob] Ciphertext : {len(ct_bob_data)/1024:.1f} KB — envoi a Alice...")
                send_data(sock, ct_bob_data)

                pos_info = json.dumps({
                    "pos": list(BOB_TRAJ[seg]),
                    "seg": seg
                }).encode()
                send_data(sock, pos_info)

                result_ct_data = recv_data(sock)
                write_file(result_ct_file, result_ct_data)

                print("[Bob] Dechiffrement du resultat final...")
                out = run([
                    BINARY_PATH, "--mode", "decrypt",
                    "--ctx", ctx_file,
                    "--sk", sk_bob_file,
                    "--ct", result_ct_file
                ])

                collision = False
                for line in out.splitlines():
                    if line.startswith("JSON_RESULT:"):
                        d = json.loads(line[len("JSON_RESULT:"):])
                        collision = d.get("collision", False)
                        break

                print(f"[Bob] Resultat : {'COLLISION' if collision else 'LIBRE'}")

            finally:
                for path in [f_path, ct_bob, result_ct_file]:
                    try:
                        os.unlink(path)
                    except OSError:
                        pass

        print("\n[Bob] Simulation terminee.")

    finally:
        if sock is not None:
            sock.close()

        for path in [ctx_file, pk_bob_file, sk_bob_file, emk_bob_file, esk_bob_file, btk_bob_file, swkfc_bob_file]:            
            try:
                os.unlink(path)
            except OSError:
                pass

if __name__ == "__main__":
    main()
