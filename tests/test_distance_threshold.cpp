/**
 * test_distance_threshold.cpp
 * ===========================
 * Tests FHE pour la détection de collision par distance ≤ seuil.
 * Vérifie : computeDistancesBatchTemporal, applyTemporalMask, detectCollisionInHorizon.
 *
 * Compile : cmake --build build --target test_distance_threshold
 * Run     : ./build/test_distance_threshold
 */

#include "engine.hpp"
#include "geometry.hpp"
#include "types.hpp"
#include <iostream>
#include <string>
#include <cmath>

// ─── Mini framework ───────────────────────────────────────────────────────────
static int g_total = 0, g_passed = 0, g_failed = 0;

static void CHECK(bool cond, const std::string& label) {
    ++g_total;
    if (cond) { ++g_passed; std::cout << "  [PASS] " << label << "\n"; }
    else       { ++g_failed; std::cout << "  [FAIL] " << label << "\n"; }
}
static void SECTION(const std::string& t) { std::cout << "\n=== " << t << " ===\n"; }

// ─── 1. computeDistancesBatchTemporal ────────────────────────────────────────
static void test_distances(CryptoEngine& engine, GeometryEngine& geo) {
    SECTION("computeDistancesBatchTemporal — distances² chiffrées");

    // Deux drones au même point → distance² = 0
    Trajectory3D a1 = {{0,0,0}, {10,10,10}};
    Trajectory3D b1 = {{0,0,0}, {10,10,10}};
    auto ct1 = geo.computeDistancesBatchTemporal(a1, b1);
    auto v1  = engine.decryptVector(ct1, 2);
    CHECK(std::abs(v1[0]) < 1.0, "Distance² = 0 quand drones au même point (t=0)");
    CHECK(std::abs(v1[1]) < 1.0, "Distance² = 0 quand drones au même point (t=1)");

    // Deux drones à distance connue : (0,0,0) et (3,4,0) → dist²=25
    Trajectory3D a2 = {{0,0,0}};
    Trajectory3D b2 = {{3,4,0}};
    auto ct2 = geo.computeDistancesBatchTemporal(a2, b2);
    auto v2  = engine.decryptVector(ct2, 1);
    CHECK(std::abs(v2[0] - 25.0) < 2.0, "Distance² = 25 pour (0,0,0)↔(3,4,0)");

    // Distance (0,0,0) → (0,0,10) → dist²=100
    Trajectory3D a3 = {{0,0,0}};
    Trajectory3D b3 = {{0,0,10}};
    auto ct3 = geo.computeDistancesBatchTemporal(a3, b3);
    auto v3  = engine.decryptVector(ct3, 1);
    CHECK(std::abs(v3[0] - 100.0) < 2.0, "Distance² = 100 pour (0,0,0)↔(0,0,10)");
}

// ─── 2. applyTemporalMask ────────────────────────────────────────────────────
static void test_mask(CryptoEngine& engine, GeometryEngine& geo) {
    SECTION("applyTemporalMask — masquage des pas de temps");

    // 4 positions, horizon = 2 → slots [0,1] actifs, [2,3] = 0
    Trajectory3D a = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
    Trajectory3D b = {{5,0,0},{5,0,0},{5,0,0},{5,0,0}};  // dist²=25 partout

    auto ct_dist = geo.computeDistancesBatchTemporal(a, b);
    auto ct_mask = geo.applyTemporalMask(ct_dist, 4, 2);
    auto v = engine.decryptVector(ct_mask, 4);

    CHECK(v[0] > 10.0, "Slot 0 actif (dans l'horizon)");
    CHECK(v[1] > 10.0, "Slot 1 actif (dans l'horizon)");
    CHECK(std::abs(v[2]) < 2.0, "Slot 2 masqué (hors horizon)");
    CHECK(std::abs(v[3]) < 2.0, "Slot 3 masqué (hors horizon)");
}

// ─── 3. detectCollisionInHorizon ─────────────────────────────────────────────
static void test_detect(CryptoEngine& engine, GeometryEngine& geo) {
    SECTION("detectCollisionInHorizon — distance² <= seuil² (TFHE)");

    const double SEUIL = 15.0;  // seuil de sécurité

    // Cas 1 : drones très proches (dist=5 < seuil=15) → danger = 1
    Trajectory3D a_close = {{0,0,0}};
    Trajectory3D b_close = {{3,4,0}};  // dist=5
    auto ct_d1  = geo.computeDistancesBatchTemporal(a_close, b_close);
    auto ct_m1  = geo.applyTemporalMask(ct_d1, 1, 1);
    auto ct_r1  = geo.detectCollisionInHorizon(ct_m1, SEUIL);
    auto v1     = engine.decryptVector(ct_r1, 1);
    CHECK(v1[0] > 0.5, "dist=5 < seuil=15 → danger détecté (≈1)");

    // Cas 2 : drones loin (dist=30 > seuil=15) → danger = 0
    Trajectory3D a_far = {{0,0,0}};
    Trajectory3D b_far = {{30,0,0}};  // dist=30
    auto ct_d2  = geo.computeDistancesBatchTemporal(a_far, b_far);
    auto ct_m2  = geo.applyTemporalMask(ct_d2, 1, 1);
    auto ct_r2  = geo.detectCollisionInHorizon(ct_m2, SEUIL);
    auto v2     = engine.decryptVector(ct_r2, 1);
    CHECK(v2[0] < 0.5, "dist=30 > seuil=15 → pas de danger (≈0)");

    // Cas 3 : distance exactement égale au seuil (dist=15) → danger = 1
    Trajectory3D a_eq = {{0,0,0}};
    Trajectory3D b_eq = {{15,0,0}};  // dist=15 = seuil
    auto ct_d3  = geo.computeDistancesBatchTemporal(a_eq, b_eq);
    auto ct_m3  = geo.applyTemporalMask(ct_d3, 1, 1);
    auto ct_r3  = geo.detectCollisionInHorizon(ct_m3, SEUIL);
    auto v3     = engine.decryptVector(ct_r3, 1);
    CHECK(v3[0] > 0.5, "dist=15 = seuil → danger détecté (bord)");

}

// ─── main ────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "========================================\n";
    std::cout << "  Tests FHE — Distance <= Seuil (TFHE)  \n";
    std::cout << "========================================\n";

    std::cout << "\n[*] Initialisation CryptoEngine...\n";
    CryptoEngine engine;
    CryptoEngine::Config cfg;
    cfg.batchSize    = 64;
    cfg.switchValues = 16;
    cfg.logQ_ccLWE   = 25;
    engine.initialize(cfg);
    engine.setupSchemeSwitching();
    std::cout << "[*] Prêt.\n";

    GeometryEngine geo(&engine);

    test_distances(engine, geo);
    test_mask(engine, geo);
    test_detect(engine, geo);

    std::cout << "\n========================================\n";
    std::cout << "  Résultat : " << g_passed << "/" << g_total << " tests passés";
    if (g_failed > 0) std::cout << "  (" << g_failed << " ECHECS)";
    std::cout << "\n========================================\n";

    return (g_failed == 0) ? 0 : 1;
}
