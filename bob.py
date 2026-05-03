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
STATE_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone_state.json")
ALICE_HOST  = "127.0.0.1"
ALICE_PORT  = 9001

ALICE_TRAJ = [
    (0, 0, 30),
    (10, 10, 30)
]  # used only for visualisation

BOB_TRAJ = [
    (0, 50, 30),
    (10, 40, 30)
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

def cpp_send(proc, cmd):
    print(f"[Bob][C++ CMD] {cmd}", flush=True)
    proc.stdin.write(cmd + "\n")
    proc.stdin.flush()

    while True:
        line = proc.stdout.readline()
        if not line:
            raise RuntimeError("C++ server stopped unexpectedly")

        line = line.strip()
        print(f"[Bob][C++ OUT] {line}", flush=True)

        if line.startswith("OK "):
            return line

        if line.startswith("ERR "):
            raise RuntimeError(line)

def pts_to_str(pts):
    return "".join(f"({x},{y},{z})" for x, y, z in pts)

def write_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f)

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
    cpp_proc = None
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

        cpp_proc = subprocess.Popen(
            [BINARY_PATH, "--mode", "server", "--role", "bob"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        cpp_send(cpp_proc, f"LOAD_LOCAL {ctx_file} {pk_bob_file} {sk_bob_file}")
        print("[Bob] Serveur C++ pret")

        print(f"[Bob] Connexion a Alice sur {ALICE_HOST}:{ALICE_PORT}...")
        for attempt in range(15):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((ALICE_HOST, ALICE_PORT))
                break
            except ConnectionRefusedError:
                print(f"[Bob] Alice pas prete, attente... ({attempt + 1}/15)")
                time.sleep(2)
                sock.close()
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

        write_state({
            "alice": {"pos": list(ALICE_TRAJ[0]), "seg": 0, "total": len(ALICE_TRAJ) - 1},
            "bob":   {"pos": list(BOB_TRAJ[0]),   "seg": 0, "total": n},
            "collision": False, "collision_msg": "",
            "fhe_running": False, "fhe_time_ms": 0, "scheme_switches": 0,
            "finished": False,
            "traj_alice": list(ALICE_TRAJ), "traj_bob": list(BOB_TRAJ),
        })

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

            write_state({
                "alice": {"pos": list(ALICE_TRAJ[seg]), "seg": seg, "total": len(ALICE_TRAJ) - 1},
                "bob":   {"pos": list(BOB_TRAJ[seg]),   "seg": seg, "total": n},
                "collision": False, "collision_msg": "",
                "fhe_running": True, "fhe_time_ms": 0, "scheme_switches": 0,
                "finished": False,
                "traj_alice": list(ALICE_TRAJ), "traj_bob": list(BOB_TRAJ),
            })

            f_path = tempfile.NamedTemporaryFile(mode="w", suffix=".txt", delete=False).name
            ct_bob = tempfile.NamedTemporaryFile(suffix="_ct_bob.bin", delete=False).name
            result_ct_file = tempfile.NamedTemporaryFile(suffix="_ct_result.bin", delete=False).name

            try:
                with open(f_path, "w") as f:
                    f.write(pts_to_str(seg_bob) + "\n")

                print("[Bob] Chiffrement avec pk_bob...")
                cpp_send(cpp_proc, f"ENCRYPT_PATH {f_path} {ct_bob}")

                ct_bob_data = read_file(ct_bob)
                print(f"[Bob] Ciphertext : {len(ct_bob_data)/1024:.1f} KB — envoi a Alice...")
                send_data(sock, ct_bob_data)

                result_ct_data = recv_data(sock)
                write_file(result_ct_file, result_ct_data)

                fhe_meta = json.loads(recv_data(sock).decode())

                print("[Bob] Dechiffrement du resultat final...")
                out = cpp_send(cpp_proc, f"DECRYPT_PATH {result_ct_file}")

                collision = "collision=1" in out
                print(f"[Bob] Resultat : {'COLLISION' if collision else 'LIBRE'}")
                print(f"[Bob] Temps FHE Alice — encrypt={fhe_meta.get('fhe_enc_ms',0):.0f} ms, detect={fhe_meta.get('fhe_det_ms',0):.0f} ms, total={fhe_meta.get('fhe_time_ms',0):.0f} ms")

                write_state({
                    "alice": {"pos": list(ALICE_TRAJ[seg]), "seg": seg, "total": len(ALICE_TRAJ) - 1},
                    "bob":   {"pos": list(BOB_TRAJ[seg]),   "seg": seg, "total": n},
                    "collision": collision,
                    "collision_msg": "⚠ COLLISION DETECTEE" if collision else "",
                    "fhe_running": False,
                    "fhe_time_ms": fhe_meta.get("fhe_time_ms", 0),
                    "scheme_switches": fhe_meta.get("scheme_switches", 0),
                    "finished": False,
                    "traj_alice": list(ALICE_TRAJ), "traj_bob": list(BOB_TRAJ),
                })

            finally:
                for path in [f_path, ct_bob, result_ct_file]:
                    try:
                        os.unlink(path)
                    except OSError:
                        pass

        write_state({
            "alice": {"pos": list(ALICE_TRAJ[-1]), "seg": n, "total": len(ALICE_TRAJ) - 1},
            "bob":   {"pos": list(BOB_TRAJ[-1]),   "seg": n, "total": n},
            "collision": False, "collision_msg": "",
            "fhe_running": False, "fhe_time_ms": 0, "scheme_switches": 0,
            "finished": True,
            "traj_alice": list(ALICE_TRAJ), "traj_bob": list(BOB_TRAJ),
        })
        print("\n[Bob] Simulation terminee.")

    finally:
        if cpp_proc is not None:
            try:
                cpp_send(cpp_proc, "QUIT")
                cpp_proc.wait(timeout=5)
            except Exception:
                cpp_proc.kill()

        if sock is not None:
            sock.close()

        for path in [ctx_file, pk_bob_file, sk_bob_file, emk_bob_file, esk_bob_file, btk_bob_file, swkfc_bob_file]:            
            try:
                os.unlink(path)
            except OSError:
                pass

if __name__ == "__main__":
    main()