#include "engine.hpp"
#include "geometry.hpp"
#include <iostream>
#include <vector>

int main() {
    std::cout << "=== Test Batching 3D ===" << std::endl;

    CryptoEngine engine;
    CryptoEngine::Config cfg;
    cfg.multDepth    = 17;
    cfg.scaleModSize = 50;
    cfg.batchSize    = 8;
    cfg.ringDim      = 8192;
    cfg.scaleSign    = 512;
    cfg.guardGain    = 100.0;

    engine.initialize(cfg);
    engine.setupSchemeSwitching();

    GeometryEngine geo(&engine);

    // Trajectoires de 8 instants
    Trajectory3D trajA = {
        {0,0,100}, {10,0,100}, {20,0,100}, {30,0,100},
        {40,0,100}, {50,0,100}, {60,0,100}, {70,0,100}
    };
    Trajectory3D trajB = {
        {0,0,110}, {10,0,108}, {20,0,105}, {30,0,102},
        {40,0,100}, {50,0,100}, {60,0,100}, {70,0,100}
    };

    // Partie 1 : calcul des distances en batch
    std::cout << "\n-- Calcul distances batch --" << std::endl;
    auto ct_d2 = geo.computeDistancesBatchTemporal(trajA, trajB);
    auto d2 = engine.decryptVector(ct_d2);
    for (size_t t = 0; t < d2.size(); t++) {
        std::cout << "  d²(t=" << t << ") = " << d2[t] << std::endl;
    }

    // Partie 2 : masque temporel
    std::cout << "\n-- Masque temporel (horizon=4) --" << std::endl;
    auto ct_masked = geo.applyTemporalMask(ct_d2, 8, 4);
    auto masked = engine.decryptVector(ct_masked);
    for (size_t t = 0; t < masked.size(); t++) {
        std::cout << "  masked(t=" << t << ") = " << masked[t] << std::endl;
    }

    // Partie 3 : candidates d'altitude
    std::cout << "\n-- Candidates d'altitude --" << std::endl;
    double currentZ = 100.0;
    double delta    = 20.0;
    int k           = 8;
    auto ct_cand = geo.encodeCandidateAltitudes(currentZ, delta, k);
    auto cands   = engine.decryptVector(ct_cand);
    for (size_t j = 0; j < cands.size(); j++) {
        std::cout << "  candidate[" << j << "] = " << cands[j] << "m" << std::endl;
    }

    // Partie 4 : vérifier les candidates contre un drone à 120m
    std::cout << "\n-- Disponibilité (drone voisin à 120m, seuil=15m) --" << std::endl;
    auto ct_avail = geo.checkCandidatesAgainstDrone(ct_cand, 120.0, 15.0);
    auto avail    = engine.decryptVector(ct_avail);
    for (size_t j = 0; j < avail.size(); j++) {
        std::cout << "  candidate " << cands[j] << "m : "
                  << (avail[j] > 0.5 ? "LIBRE" : "OCCUPEE") << std::endl;
    }

    // Partie 5 : meilleure altitude
    std::cout << "\n-- Meilleure altitude --" << std::endl;
    double best = geo.selectBestAltitude(ct_avail, currentZ, delta, k);
    std::cout << "  Nouvelle altitude : " << best << "m" << std::endl;

    return 0;
}
