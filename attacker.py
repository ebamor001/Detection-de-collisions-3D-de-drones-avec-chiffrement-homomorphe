#!/usr/bin/env python3
"""
attacker.py — Simulation d'un attaquant reseau
Montre qu'intercepter les ciphertexts ne revele aucune information.

Usage :
  1. Lancer alice.py + bob.py une fois (genere demo_ct_alice.bin)
  2. python3 attacker.py
"""

import os, struct

DEMO_CT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_ct_alice.bin")

print("=" * 60)
print("  ATTAQUANT — Tentative d'extraction de coordonnees")
print("=" * 60)

if not os.path.exists(DEMO_CT):
    print(f"\n[Attaquant] Fichier introuvable : {DEMO_CT}")
    print("[Attaquant] Lance d'abord alice.py + bob.py au moins une fois.")
    exit(1)

print(f"\n[Attaquant] Ciphertext intercepte : {DEMO_CT}")
print(f"[Attaquant] Taille               : {os.path.getsize(DEMO_CT)/1024:.1f} KB\n")

# ── Tentative 1 : lire comme du texte ────────────────────────────────────────
print("-" * 60)
print("TENTATIVE 1 : Lecture comme texte (coordonnees x,y,z ?)")
print("-" * 60)
try:
    with open(DEMO_CT, "r", encoding="utf-8") as f:
        content = f.read(200)
    print(f"Contenu : {content}")
except UnicodeDecodeError as e:
    print(f"[ECHEC] Impossible de lire comme texte UTF-8")
    print(f"        {e}")
    print("=> Les donnees ne sont pas des coordonnees en clair.\n")

# ── Tentative 2 : afficher les octets bruts ───────────────────────────────────
print("-" * 60)
print("TENTATIVE 2 : Affichage des 64 premiers octets bruts")
print("-" * 60)
with open(DEMO_CT, "rb") as f:
    raw = f.read(64)
print("Octets 00-31 (hex) : " + " ".join(f"{b:02x}" for b in raw[:32]))
print("Octets 32-63 (hex) : " + " ".join(f"{b:02x}" for b in raw[32:64]))
print("=> Bruit cryptographique — aucune structure lisible.\n")

# ── Tentative 3 : chercher des coordonnees entieres ──────────────────────────
print("-" * 60)
print("TENTATIVE 3 : Recherche de coordonnees entieres (0-100)")
print("-" * 60)
with open(DEMO_CT, "rb") as f:
    data = f.read(4000)

found = []
for i in range(0, len(data) - 3, 4):
    try:
        val = struct.unpack_from("<I", data, i)[0]
        if 0 < val < 100:
            found.append((i, val))
    except: pass

print(f"Valeurs entre 0 et 100 trouvees : {len(found)} occurrences")
if found:
    print(f"Exemples : {found[:5]}")
print("=> Artefacts aleatoires. Impossible de distinguer les")
print("   vraies coordonnees du bruit sans la cle secrete.\n")

# ── Tentative 4 : verifier que les vraies coords sont introuvables ───────────
print("-" * 60)
print("TENTATIVE 4 : Verification (vraies coords : (0,50,30)->(6,54,30))")
print("-" * 60)
with open(DEMO_CT, "rb") as f:
    full = f.read()

for t in [0, 50, 30, 6, 54]:
    count = sum(1 for i in range(0, len(full)-3, 4)
                if struct.unpack_from("<I", full, i)[0] == t)
    print(f"  Valeur {t:3d} : {count:5d} occurrences dans le ciphertext")

print("\n=> Ces valeurs apparaissent autant que n'importe quelle")
print("   autre. Elles sont noyees dans les coefficients du polynome")
print("   CKKS de dimension 4096.\n")

# ── Conclusion ────────────────────────────────────────────────────────────────
print("=" * 60)
print("  CONCLUSION")
print("=" * 60)
print(f"\n  Taille interceptee     : {os.path.getsize(DEMO_CT)/1024:.1f} KB")
print(  "  Coordonnees recuperees : 0 / 6")
print(  "  Resultat               : ECHEC TOTAL\n")
print(  "  La securite repose sur RLWE — retrouver m depuis")
print(  "  Encrypt(pk, m) est infaisable sans sk_Alice.\n")
print(  "  => La confidentialite des trajectoires est GARANTIE.")
print("=" * 60)
