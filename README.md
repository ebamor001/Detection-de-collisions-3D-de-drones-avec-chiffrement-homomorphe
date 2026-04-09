# Détection de Collisions 3D de Drones avec Chiffrement Homomorphe

Projet sur le calcul sécurisé de données sensibles dans des systèmes distribués (IoT, cloud, drones), basé sur le chiffrement homomorphe — avec un accent particulier sur **CKKS** et **TFHE** pour permettre des calculs sans révélation des données.

---

## Prérequis

| Dépendance | Version minimale | Installation |
|---|---|---|
| CMake | >= 3.16 | `sudo apt install cmake` |
| GCC / G++ | >= 11 (C++17) | `sudo apt install g++` |
| OpenFHE | >= 1.1 | Voir ci-dessous |
| OpenMP | — | `sudo apt install libomp-dev` |
| Python | >= 3.10 | `sudo apt install python3` |
| matplotlib / numpy | — | `pip3 install matplotlib numpy` |

### Installer OpenFHE

```bash
git clone https://github.com/openfheorg/openfhe-development.git
cd openfhe-development
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

## Compilation

```bash
# Depuis la racine du projet
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

Le binaire compilé sera disponible dans `./build/drone_collision`.

---

## Lancer le binaire C++

### Mode test rapide (chemins aléatoires)

```bash
./build/drone_collision
```

### Scénarios prédéfinis

```bash
# Scénario 1 : COLLISION garantie (route1 + route2, même altitude z=10)
./build/drone_collision --scenario 1

# Scénario 2 : PAS DE COLLISION (route1 + route3, altitudes différentes z=10 vs z=50)
./build/drone_collision --scenario 2

# Scénario 3 : Trajectoires complexes (carré + zigzag)
./build/drone_collision --scenario 3
```

### Fichiers personnalisés

```bash
./build/drone_collision --path1 data/cryptroute1.txt --path2 data/cryptroute2.txt
./build/drone_collision --path1 data/cryptroute4.txt --path2 data/cryptroute5.txt
```

### Autres options

```bash
# Afficher l'aide
./build/drone_collision --help

# Sortie JSON (pour intégration serveur)
./build/drone_collision --scenario 1 --json
```

---

## Lancer le serveur web

Le serveur Python fournit une API REST locale et sert l'interface web.

```bash
# Depuis la racine du projet
python3 server.py
```

### Endpoints API

**POST** `/api/analyze`

Requête :
```json
{
  "path1": [[0, 0, 10], [50, 50, 10], [99, 0, 10]],
  "path2": [[0, 50, 10], [50,  0, 10], [99, 50, 10]]
}
```

Réponse :
```json
{
  "collision": true,
  "segments_tested": 4,
  "scheme_switches": 6,
  "scheme_switches_no_batch": 36,
  "batching_gain": 6,
  "time_ms": 49920.0,
  "details": [{"seg1": 0, "seg2": 0}]
}
```

**GET** `/api/health`
```json
{ "status": "ok", "binary": true }
```

> Le serveur fonctionne en **mode simulation** : la détection est faite en Python (< 1 ms), les métriques FHE sont calculées à partir du modèle réel (820 ms/scheme-switch, 45s keygen).

---

## Fichiers de données

Format des fichiers : `(x,y,z)(x,y,z)...` avec `-99 <= x,y,z <= 100`

| Fichier | Description | Résultat avec route1 |
|---|---|---|
| `cryptroute1.txt` | Drone A, trajectoire en V, z=10 | — |
| `cryptroute2.txt` | Drone B, V inversé, z=10 | **COLLISION** |
| `cryptroute3.txt` | Drone C, même XY que A, z=50 | Pas de collision |
| `cryptroute4.txt` | Drone D, trajectoire carrée, z=15 | — |
| `cryptroute5.txt` | Drone E, trajectoire zigzag, z=20 | — |

### Regénérer les fichiers

```bash
cd data/
bash create_example_paths.sh
```

### Créer un fichier personnalisé

```bash
# Format : (x,y,z)(x,y,z)... sur une seule ligne
echo "(0,0,10)(30,50,10)(70,20,10)(99,60,10)" > data/ma_route.txt
./build/drone_collision --path1 data/ma_route.txt --path2 data/cryptroute2.txt
```

---

## Générer les courbes de performance

```bash
python3 plot_batching.py
```

Génère `batching_analysis.png` avec 4 graphiques :

| Graphique | Description |
|---|---|
| Scheme-switches vs N | Batching constant (6 SS) vs sans batching (N×9 SS) |
| Temps estimé vs N | Gain de temps croissant avec N |
| Gain en × vs N | Gain linéaire : ×1.5 à N=1, ×75 à N=50 |
| Bar chart N=16 | 144 SS → 6 SS = **×24** pour notre test |

---

## Cas de tests

### Test 1 — Collision simple (même altitude)

```bash
./build/drone_collision --scenario 1
# Attendu : COLLISION DETECTEE
# route1 = V descendant, route2 = V inversé, z=10 identique
```

### Test 2 — Séparation par altitude

```bash
./build/drone_collision --scenario 2
# Attendu : AUCUNE COLLISION
# route1 à z=10, route3 à z=50 → coplanarité impossible
```

### Test 3 — Trajectoires complexes

```bash
./build/drone_collision --scenario 3
# route4 = carré (z=15), route5 = zigzag (z=20) → altitudes différentes
```

### Test 4 — Sorties comparées (clair vs chiffré)

```bash
./build/drone_collision --scenario 1
# Vérifier que :
# "Clear intersections found" == "COLLISION DETECTEE"
# Les scheme-switches affichés correspondent au modèle batching
```

### Test 5 — API serveur

```bash
# Terminal 1 : lancer le serveur
python3 server.py

# Terminal 2 : tester l'API
curl -X POST http://localhost:8765/api/analyze \
  -H "Content-Type: application/json" \
  -d '{"path1":[[0,0,10],[50,50,10],[99,0,10]],"path2":[[0,50,10],[50,0,10],[99,50,10]]}'
```

---

## Paramètres OpenFHE

| Paramètre | Valeur démo | Valeur production |
|---|---|---|
| Ring dimension | 8192 | 8192 |
| MultDepth | 20 | 20 |
| ScaleModSize | 50 bits | 50 bits |
| BatchSize (slots) | 64 | 4096 |
| switchValues | 64 | 4096 |
| Temps setup | ~1-2 min | ~10-30 min |
