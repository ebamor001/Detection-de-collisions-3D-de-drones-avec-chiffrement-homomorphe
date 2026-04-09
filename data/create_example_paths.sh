#!/bin/bash
# Génère les fichiers de trajectoires 3D de drones
# Format : (x,y,z) avec -99 <= x,y,z <= 100
# Lancer depuis le dossier data/ :  bash create_example_paths.sh

cd "$(dirname "$0")"

# ── Route 1 : Drone A à altitude 10 (X en forme de V) ────────────────────────
echo "(0,0,10)(50,50,10)(99,0,10)" > cryptroute1.txt
echo "  cryptroute1.txt : Drone A, altitude z=10, trajectoire en V"

# ── Route 2 : Drone B à altitude 10 (croise le Drone A → COLLISION) ──────────
echo "(0,50,10)(50,0,10)(99,50,10)" > cryptroute2.txt
echo "  cryptroute2.txt : Drone B, altitude z=10 → collision avec route1"

# ── Route 3 : Drone C à altitude 50 (au dessus de A, pas de collision) ────────
echo "(0,0,50)(50,50,50)(99,0,50)" > cryptroute3.txt
echo "  cryptroute3.txt : Drone C, altitude z=50 → aucune collision avec route1"

# ── Route 4 : Drone D, trajectoire carrée à altitude 15 ───────────────────────
echo "(10,10,15)(10,90,15)(90,90,15)(90,10,15)(10,10,15)" > cryptroute4.txt
echo "  cryptroute4.txt : Drone D, carré, altitude z=15"

# ── Route 5 : Drone E, zigzag complexe à altitude 20 ──────────────────────────
echo "(-50,-50,20)(0,50,20)(50,-50,20)(99,50,20)(50,99,20)(-50,50,20)(-99,-50,20)" > cryptroute5.txt
echo "  cryptroute5.txt : Drone E, zigzag, altitude z=20"

# ── Fichier multi-routes : 3 trajectoires, une par ligne ──────────────────────
cat > multiple_routes.txt << 'EOF'
(0,0,10)(20,20,10)(40,0,10)
(10,30,10)(30,10,10)(50,30,10)
(-20,-20,30)(0,0,30)(20,-20,30)
EOF
echo "  multiple_routes.txt : 3 routes (ligne 1&2 altitude z=10 → collision, ligne 3 z=30)"

echo ""
echo "Tous les fichiers créés dans data/"
echo "Format valide : (x,y,z) avec -99 <= x,y,z <= 100"
echo ""
echo "Exemples d'utilisation :"
echo "  ./build/drone_collision --path1 data/cryptroute1.txt --path2 data/cryptroute2.txt"
echo "  ./build/drone_collision --path1 data/cryptroute1.txt --path2 data/cryptroute3.txt"
