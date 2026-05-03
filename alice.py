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
ALICE_PORT = 9001

ALICE_TRAJ = [
    (0, 50, 30), (6, 54, 30)
]

BOB_TRAJ = [
    (100, 0, 30), (98, 6, 30), (95, 13, 30), (90, 20, 30), (83, 26, 30),
    (75, 33, 30)
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

def cpp_send(proc, cmd):
    print(f"[Alice][C++ CMD] {cmd}", flush=True)
    proc.stdin.write(cmd + "\n")
    proc.stdin.flush()

    while True:
        line = proc.stdout.readline()
        if not line:
            raise RuntimeError("C++ server stopped unexpectedly")

        line = line.strip()
        print(f"[Alice][C++ OUT] {line}", flush=True)

        if line.startswith("OK "):
            return line

        if line.startswith("ERR "):
            raise RuntimeError(line)

def pts_to_str(pts):
    return "".join(f"({x},{y},{z})" for x, y, z in pts)

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

        cpp_proc = None
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


            cpp_proc = subprocess.Popen(
                [BINARY_PATH, "--mode", "server", "--role", "alice"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            cpp_send(
                cpp_proc,
                f"LOAD_REMOTE {ctx_file} {pk_bob_file} {emk_bob_file} {esk_bob_file} {btk_bob_file} {swkfc_bob_file}"
            )

            print(f"[Alice] Contexte recu : {len(ctx_data)/1024:.1f} KB")
            print(f"[Alice] PK Bob       : {len(pk_bob_data)/1024:.1f} KB")

            total_fhe_ms = 0.0

            n = len(ALICE_TRAJ) - 1
            for seg in range(1, n + 1):
                print(f"\n[Alice] === Segment {seg}/{n} ===")

                seg_alice = [ALICE_TRAJ[seg - 1], ALICE_TRAJ[seg]]

                f_alice_path = tempfile.NamedTemporaryFile(mode="w", suffix=".txt", delete=False).name
                f_alice_ct = tempfile.NamedTemporaryFile(suffix="_ct_alice.bin", delete=False).name
                f_bob_ct = tempfile.NamedTemporaryFile(suffix="_ct_bob.bin", delete=False).name
                f_result_ct = tempfile.NamedTemporaryFile(suffix="_ct_result.bin", delete=False).name

                try:
                    with open(f_alice_path, "w") as f:
                        f.write(pts_to_str(seg_alice) + "\n")

                    print("[Alice] Chiffrement du segment d'Alice avec pk_bob...", flush=True)
                    t_enc = time.time()
                    cpp_send(cpp_proc, f"ENCRYPT_PATH {f_alice_path} {f_alice_ct}")
                    ms_enc = (time.time() - t_enc) * 1000.0
                    send_data(conn, str(seg).encode())

                    bob_ct_data = recv_data(conn)
                    write_file(f_bob_ct, bob_ct_data)
                    print(f"[Alice] Ciphertext Bob recu : {len(bob_ct_data)/1024:.1f} KB")

                    print("[Alice] Calcul FHE (DETECT_PATH)...", flush=True)
                    t_det = time.time()

                    out = cpp_send(
                        cpp_proc,
                        f"DETECT_PATH {f_alice_ct} {f_bob_ct} {f_result_ct} 0"
                    )

                    ms_det = (time.time() - t_det) * 1000.0
                    ms = ms_enc + ms_det
                    total_fhe_ms += ms

                    ss = 0
                    if "scheme_switches=" in out:
                        ss = int(out.split("scheme_switches=")[1].split()[0])
                    result_ct_data = read_file(f_result_ct)

                    fhe_meta = json.dumps({
                        "fhe_time_ms": round(ms, 1),
                        "fhe_enc_ms": round(ms_enc, 1),
                        "fhe_det_ms": round(ms_det, 1),
                        "scheme_switches": ss
                    }).encode()
                    send_data(conn, result_ct_data)
                    send_data(conn, fhe_meta)

                    print(f"[Alice] Segment {seg}: encrypt={ms_enc:.0f} ms, detect={ms_det:.0f} ms, total={ms:.0f} ms ({ss} SS)")

                finally:
                    for path in [f_alice_path, f_alice_ct, f_bob_ct, f_result_ct]:
                        try:
                            os.unlink(path)
                        except OSError:
                            pass

                time.sleep(0.5)

            print(f"\n[Alice] Temps FHE total (encrypt + detect, tous segments) : {total_fhe_ms:.0f} ms ({total_fhe_ms/1000:.2f} s)")
            print("\n[Alice] Simulation terminee.")

        finally:
            if cpp_proc is not None:
                try:
                    cpp_send(cpp_proc, "QUIT")
                    cpp_proc.wait(timeout=5)
                except Exception:
                    cpp_proc.kill()
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