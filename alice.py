#!/usr/bin/env python3
"""
alice.py — Drone Alice (serveur)

Protocole corrige :
- Alice attend Bob
- Bob envoie ctx + pk  + emk + esk
- Alice chiffre son segment avec pk_bob
- Alice lance detect sur ct_alice et ct_bob
- Alice renvoie le resultat chiffre a Bob
- Alice ne dechiffre jamais
"""

import socket
import struct
import subprocess
import tempfile
import os
import json
import time

BINARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone_state.json")
ALICE_PORT = 9001

ALICE_TRAJ = [
    (0, 50, 30), (6, 54, 30), (13, 58, 30), (20, 61, 30), (26, 64, 30),
    (33, 67, 30), (40, 69, 30), (46, 69, 30), (53, 69, 30), (60, 69, 30),
    (66, 67, 30), (73, 64, 30), (80, 61, 30), (86, 58, 30), (93, 54, 30), (100, 50, 30)
]

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
    print("\n[Alice][RUN] " + " ".join(cmd), flush=True)

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)

    print(f"[Alice][RC] {result.returncode}", flush=True)

    if result.stdout:
        print("\n[Alice][STDOUT]")
        print(result.stdout, flush=True)

    if result.stderr:
        print("\n[Alice][STDERR]")
        print(result.stderr, flush=True)

    if result.returncode != 0:
        print("[Alice] La commande a plante ici.", flush=True)
        raise RuntimeError("Commande echouee")

    return result.stdout


def pts_to_str(pts):
    return "".join(f"({x},{y},{z})" for x, y, z in pts)

def write_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f)

def main():
    print("=" * 55)
    print("  ALICE — Drone 1 (port 9001)")
    print("=" * 55)

    ctx_file = tempfile.NamedTemporaryFile(suffix="_ctx.bin", delete=False).name
    pk_bob_file = tempfile.NamedTemporaryFile(suffix="_pk_bob.bin", delete=False).name
    emk_bob_file = tempfile.NamedTemporaryFile(suffix="_emk_bob.bin", delete=False).name
    esk_bob_file = tempfile.NamedTemporaryFile(suffix="_esk_bob.bin", delete=False).name
    btk_bob_file = tempfile.NamedTemporaryFile(suffix="_btk_bob.bin", delete=False).name
    swkfc_bob_file = tempfile.NamedTemporaryFile(suffix="_swkfc_bob.bin", delete=False).name
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("", ALICE_PORT))
        server.listen(1)
        print(f"[Alice] En attente de Bob sur le port {ALICE_PORT}...")

        conn, addr = server.accept()
        print(f"[Alice] Bob connecte depuis {addr}")

        try:
            print("[Alice] Reception de ctx + pk_bob + emk_bob + esk_bob + btk_bob + swkfc_bob...")
            ctx_data = recv_data(conn)
            pk_bob_data = recv_data(conn)
            emk_bob_data = recv_data(conn)
            esk_bob_data = recv_data(conn)
            btk_bob_data = recv_data(conn)
            swkfc_bob_data = recv_data(conn)

            write_file(ctx_file, ctx_data)
            write_file(pk_bob_file, pk_bob_data)
            write_file(emk_bob_file, emk_bob_data)
            write_file(esk_bob_file, esk_bob_data)
            write_file(btk_bob_file, btk_bob_data)
            write_file(swkfc_bob_file, swkfc_bob_data)

            print(f"[Alice] Contexte recu : {len(ctx_data)/1024:.1f} KB")
            print(f"[Alice] PK Bob       : {len(pk_bob_data)/1024:.1f} KB")

            state = {
                "alice": {"pos": list(ALICE_TRAJ[0]), "seg": 0, "total": len(ALICE_TRAJ) - 1},
                "bob": {"pos": list(BOB_TRAJ[0]), "seg": 0, "total": len(BOB_TRAJ) - 1},
                "collision": False,
                "collision_msg": "",
                "fhe_running": False,
                "fhe_time_ms": 0,
                "scheme_switches": 0,
                "finished": False,
                "traj_alice": list(ALICE_TRAJ),
                "traj_bob": list(BOB_TRAJ),
            }
            write_state(state)

            n = len(ALICE_TRAJ) - 1
            for seg in range(1, n + 1):
                print(f"\n[Alice] === Segment {seg}/{n} ===")

                state["alice"]["pos"] = list(ALICE_TRAJ[seg])
                state["alice"]["seg"] = seg
                state["fhe_running"] = True
                write_state(state)

                seg_alice = [ALICE_TRAJ[seg - 1], ALICE_TRAJ[seg]]

                f_alice_path = tempfile.NamedTemporaryFile(mode="w", suffix=".txt", delete=False).name
                f_alice_ct = tempfile.NamedTemporaryFile(suffix="_ct_alice.bin", delete=False).name
                f_bob_ct = tempfile.NamedTemporaryFile(suffix="_ct_bob.bin", delete=False).name
                f_result_ct = tempfile.NamedTemporaryFile(suffix="_ct_result.bin", delete=False).name

                try:
                    with open(f_alice_path, "w") as f:
                        f.write(pts_to_str(seg_alice) + "\n")

                    
                    print("[Alice] Chiffrement du segment d'Alice avec pk_bob...", flush=True)
                    run([
                        BINARY_PATH, "--mode", "encrypt",
                        "--ctx", ctx_file,
                        "--pk", pk_bob_file,
                        "--path", f_alice_path,
                        "--out", f_alice_ct
                    ])
                    send_data(conn, str(seg).encode())

                    bob_ct_data = recv_data(conn)
                    write_file(f_bob_ct, bob_ct_data)
                    print(f"[Alice] Ciphertext Bob recu : {len(bob_ct_data)/1024:.1f} KB")

                    bob_pos_data = recv_data(conn)
                    bob_pos = json.loads(bob_pos_data.decode())
                    state["bob"]["pos"] = bob_pos["pos"]
                    state["bob"]["seg"] = bob_pos["seg"]

                    print("[Alice] Calcul FHE...", flush=True)
                    t0 = time.time()

                    out = run([
                        BINARY_PATH, "--mode", "detect",
                        "--ctx", ctx_file,
                        "--pk", pk_bob_file,
                        "--emk", emk_bob_file,
                        "--esk", esk_bob_file,
                        "--btk", btk_bob_file,
                        "--swkfc", swkfc_bob_file,
                        "--ct1", f_alice_ct,
                        "--ct2", f_bob_ct,
                        "--out", f_result_ct
                    ])

                    ms = (time.time() - t0) * 1000.0

                    ss = 0
                    for line in out.splitlines():
                        if line.startswith("JSON_RESULT:"):
                            d = json.loads(line[len("JSON_RESULT:"):])
                            ss = d.get("scheme_switches", 0)
                            break
                    result_ct_data = read_file(f_result_ct)
                    send_data(conn, result_ct_data)

                    print(f"[Alice] Resultat chiffre envoye a Bob ({ms:.0f} ms, {ss} SS)")

                    state["collision"] = False
                    state["collision_msg"] = ""
                    state["fhe_running"] = False
                    state["fhe_time_ms"] = round(ms, 1)
                    state["scheme_switches"] = ss
                    write_state(state)

                finally:
                    for path in [f_alice_path, f_alice_ct, f_bob_ct, f_result_ct]:
                        try:
                            os.unlink(path)
                        except OSError:
                            pass

                time.sleep(0.5)

            state["finished"] = True
            write_state(state)
            print("\n[Alice] Simulation terminee.")

        finally:
            conn.close()
            server.close()

    finally:
        for path in [ctx_file, pk_bob_file, emk_bob_file, esk_bob_file, btk_bob_file, swkfc_bob_file]:
            try:
                os.unlink(path)
            except OSError:
                pass

if __name__ == "__main__":
    main()
