# Détection de collisions 3D de drones — Chiffrement Homomorphe

## Compilation

```bash
cd build && cmake .. && make -j4
```

---

## Lancement — 4 terminaux

**Terminal 1 — Alice (lancer en premier)**
```bash
python3 alice.py
```

**Terminal 2 — Bob (lancer après Alice)**
```bash
python3 bob.py
```

**Terminal 3 — Serveur web**
```bash
python3 server.py
```

---

## Démo web

Ouvrir dans le navigateur après avoir lancé les 4 terminaux :

```
http://localhost:8765/web/live.html
```

---

## Ordre important

```
1. alice.py   → génère le contexte FHE et attend Bob
2. bob.py     → se connecte à Alice et chiffre ses segments
3. server.py  → sert la page web
4. navigateur → http://localhost:8765/web/live.html
```
