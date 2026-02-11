#!/bin/bash

# Créer les fichiers de données d'exemple (format du papier original)

# Route 1 - Chemin simple
echo "(100,1)(300,5)(400,400)(800,400)(800,1)(400,1)(1,1)" > cryptroute1.txt

# Route 2 - Chemin qui intersecte avec Route 1
echo "(50,200)(450,200)(450,50)(750,50)(750,450)(50,450)" > cryptroute2.txt

# Route 3 - Chemin court pour tests
echo "(0,0)(50,50)(100,0)" > cryptroute3.txt

# Route 4 - Chemin avec segments horizontaux/verticaux
echo "(10,10)(10,90)(90,90)(90,10)(10,10)" > cryptroute4.txt

# Route 5 - Chemin en zigzag
echo "(-50,-50)(0,50)(50,-50)(99,50)(50,99)(-50,50)(-99,-50)" > cryptroute5.txt

# Routes multiples dans un fichier
cat > multiple_routes.txt << EOF
(0,0)(20,20)(40,0)
(10,30)(30,10)(50,30)
(-20,-20)(0,0)(20,-20)
EOF

echo "Fichiers de test créés dans data/"
echo "Format: (x1,y1)(x2,y2)... avec -99 <= x,y <= 99"