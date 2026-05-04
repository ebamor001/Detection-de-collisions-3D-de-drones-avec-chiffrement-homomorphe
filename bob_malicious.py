#!/usr/bin/env python3
"""
bob_malicious.py — Bob malveillant tente de dechiffrer avec de fausses cles
Montre qu'utiliser un contexte different produit une erreur ou un resultat faux.

Usage :
  1. Lancer alice.py + bob.py une fois (genere demo_ct_alice.bin, demo_ctx.bin...)
  2. python3 bob_malicious.py
"""

import subprocess, tempfile, os, json, time

BINARY   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "drone_fhe")
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEMO_CT  = os.path.join(BASE_DIR, "demo_ct_alice.bin")
# detect_encrypted attend un prefixe, pas un fichier complet
# demo_ct_alice.bin est le fichier _p1x — le prefixe est demo_ct_alice
DEMO_CT_PREFIX = os.path.join(BASE_DIR, "demo_ct_alice")

print("=" * 60)
print("  BOB MALVEILLANT — Tentative de dechiffrement non autorise")
print("=" * 60)

if not os.path.exists(DEMO_CT):
    print(f"\n[Bob malveillant] Fichier introuvable : {DEMO_CT}")
    print("[Bob malveillant] Lance d'abord alice.py + bob.py au moins une fois.")
    exit(1)

print(f"\n[Bob malveillant] Ciphertext d'Alice intercepte : {DEMO_CT}")
print(f"[Bob malveillant] Taille : {os.path.getsize(DEMO_CT)/1024:.1f} KB")

# ── Etape 1 : Bob genere SES PROPRES cles (differentes de celles d'Alice) ─────
print("\n[Bob malveillant] Je genere MES PROPRES cles FHE...")
print("[Bob malveillant] (Contexte completement different de celui d'Alice)")

fake_ctx = tempfile.NamedTemporaryFile(suffix="_fake_ctx.bin", delete=False).name
fake_sk  = tempfile.NamedTemporaryFile(suffix="_fake_sk.bin",  delete=False).name
fake_pk  = tempfile.NamedTemporaryFile(suffix="_fake_pk.bin",  delete=False).name
fake_emk = tempfile.NamedTemporaryFile(suffix="_fake_emk.bin", delete=False).name
fake_esk = tempfile.NamedTemporaryFile(suffix="_fake_esk.bin", delete=False).name

r = subprocess.run([BINARY, "--mode", "init",
    "--ctx", fake_ctx, "--pk", fake_pk,
    "--emk", fake_emk, "--esk", fake_esk, "--sk", fake_sk],
    capture_output=True, text=True, timeout=300)

if r.returncode != 0:
    print(f"[Bob malveillant] Erreur init : {r.stderr[:100]}")
    exit(1)
print("[Bob malveillant] Mes fausses cles sont pretes.\n")

# ── Etape 2 : Bob chiffre son propre segment avec SES cles ────────────────────
print("[Bob malveillant] Je chiffre mon segment avec MES cles (pas celles d'Alice)...")
f_bob_path = tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False).name
f_ct_bob   = tempfile.NamedTemporaryFile(suffix="_fake_ct_bob.bin", delete=False).name

with open(f_bob_path, 'w') as f:
    f.write("(100,0,30)(98,6,30)\n")

r = subprocess.run([BINARY, "--mode", "encrypt",
    "--ctx", fake_ctx, "--pk", fake_pk,
    "--path", f_bob_path, "--out", f_ct_bob],
    capture_output=True, text=True, timeout=300)

if r.returncode != 0:
    print(f"[Bob malveillant] Erreur encrypt : {r.stderr[:100]}")
else:
    print("[Bob malveillant] Segment chiffre avec mes fausses cles.\n")

# ── Etape 3 : Tentative de calcul avec contextes incompatibles ────────────────
print("-" * 60)
print("ATTAQUE : Calcul FHE avec ct_alice (contexte Alice)")
print("          + ct_bob (contexte Bob malveillant) = INCOMPATIBLE")
print("-" * 60)
print("[Bob malveillant] Lancement de detect_encrypted...")

t0 = time.time()
r = subprocess.run([BINARY, "--mode", "detect_encrypted",
    "--ctx", fake_ctx,
    "--emk", fake_emk,
    "--esk", fake_esk,
    "--ct1", DEMO_CT_PREFIX,  # prefixe des ciphertexts d'Alice
    "--ct2", f_ct_bob],       # prefixe des ciphertexts de Bob (faux contexte)
    capture_output=True, text=True, timeout=300)
ms = (time.time() - t0) * 1000

print(f"\n[Bob malveillant] Code retour : {r.returncode}")
print(f"[Bob malveillant] Temps       : {ms:.0f} ms")

if r.returncode != 0:
    err = r.stderr[:400] if r.stderr else r.stdout[:400]
    print(f"\n[Bob malveillant] ERREUR CRYPTOGRAPHIQUE :")
    print(f"  {err}")
    print("\n=> Le programme plante : les ciphertexts appartiennent")
    print("   a des anneaux cyclotomiques differents.")
    print("   Ils sont mathematiquement incompatibles.")
else:
    print(f"\n[Bob malveillant] Sortie brute :")
    print(r.stdout[-300:])
    for line in r.stdout.splitlines():
        if line.startswith("JSON_RESULT:"):
            try:
                d = json.loads(line[len("JSON_RESULT:"):])
                print(f"\n[Bob malveillant] Resultat obtenu  : collision={d.get('collision')}")
                print( "[Bob malveillant] Resultat REEL    : INCONNU pour Bob")
                print( "[Bob malveillant] Bob ne sait pas  : si c'est vrai ou faux")
            except: pass
    print("\n=> Meme si un resultat est produit, il est aleatoire.")
    print("   Bob ne peut PAS savoir s'il correspond a la realite.")

# ── Conclusion ────────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("  CONCLUSION")
print("=" * 60)
print()
print("  Bob avait : ciphertext d'Alice + ses propres cles")
print()
print("  Ce qu'il n'avait PAS : sk_Alice (cle secrete d'Alice)")
print()
print("  Resultat :")
print("  - Erreur cryptographique (contextes incompatibles)")
print("    OU resultat booleeen sans signification")
print("  - Coordonnees d'Alice : TOUJOURS INCONNUES")
print()
print("  => La cle secrete ne quitte jamais Alice.")
print("  => La confidentialite des trajectoires est GARANTIE.")
print("=" * 60)

# Nettoyage
for f in [fake_ctx, fake_sk, fake_pk, fake_emk, fake_esk, f_bob_path, f_ct_bob]:
    try: os.unlink(f)
    except: pass
