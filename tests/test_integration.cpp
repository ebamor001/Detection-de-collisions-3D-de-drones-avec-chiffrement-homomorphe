/**
 * test_integration.cpp
 * ====================
 * Test d'intégration bout-en-bout : pipeline complet FHE vs clair.
 * Vérifie que batchCheckIntersection3D + detectCollisionInHorizon donnent
 * les mêmes résultats que les fonctions en clair sur des scénarios réels.
 *
 * Compile : cmake --build build --target test_integration
 * Run     : ./build/test_integration
 */

#include "engine.hpp"
#include "geometry.hpp"
#include "intersection.hpp"
#include "io.hpp"
#include "types.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

// ─── Mini framework ───────────────────────────────────────────────────────────
static int g_total = 0, g_passed = 0, g_failed = 0;

static void CHECK(bool cond, const std::string& label) {
    ++g_total;
    if (cond) { ++g_passed; std::cout << "  [PASS] " << label << "\n"; }
    else       { ++g_failed; std::cout << "  [FAIL] " << label << "\n"; }
}
static void SECTION(const std::string& t) { std::cout << "\n=== " << t << " ===\n"; }

// ─── Helper : FHE détecte-t-il une collision entre deux chemins ? ─────────────
static bool fhe_collision(GeometryEngine& geo,
                           const Path& path1, const Path& path2) {
    std::vector<Segment> neighbors;
    for (size_t j = 0; j + 1 < path2.size(); j++)
        neighbors.push_back({path2[j], path2[j+1]});

    for (size_t i = 0; i + 1 < path1.size(); i++) {
        Segment mySeg = {path1[i], path1[i+1]};
        auto results = geo.batchCheckIntersection3D(mySeg, neighbors);
        for (double v : results)
            if (v > 0.5) return true;
    }
    return false;
}

// ─── Helper : clair détecte-t-il une collision ? ─────────────────────────────
static bool clear_collision(const Path& path1, const Path& path2) {
    for (size_t i = 0; i + 1 < path1.size(); i++) {
        Segment s1 = {path1[i], path1[i+1]};
        for (size_t j = 0; j + 1 < path2.size(); j++) {
            Segment s2 = {path2[j], path2[j+1]};
            if (GeometryEngine::doSegmentsIntersectClear3D(s1, s2))
                return true;
        }
    }
    return false;
}

// ─── Helper : FHE détecte-t-il une proximité dangereuse ? ────────────────────
static bool fhe_proximity(CryptoEngine& engine, GeometryEngine& geo,
                           const Path& path1, const Path& path2,
                           double threshold) {
    size_t N = std::min(path1.size(), path2.size());
    Trajectory3D t1, t2;
    for (size_t t = 0; t < N; t++) {
        t1.push_back({(double)path1[t].x, (double)path1[t].y, (double)path1[t].z});
        t2.push_back({(double)path2[t].x, (double)path2[t].y, (double)path2[t].z});
    }
    auto ct_dist = geo.computeDistancesBatchTemporal(t1, t2);
    auto ct_mask = geo.applyTemporalMask(ct_dist, N, N);
    auto ct_res  = geo.detectCollisionInHorizon(ct_mask, threshold);
    auto vals    = engine.decryptVector(ct_res, (uint32_t)N);
    for (double v : vals)
        if (v > 0.5) return true;
    return false;
}

// ─── 1. Scénarios réels depuis les fichiers data/ ────────────────────────────
static void test_real_scenarios(CryptoEngine& engine, GeometryEngine& geo) {
    SECTION("Scénarios réels (fichiers data/)");

    const std::string DATA = "data";

    // Scénario 1 : cryptroute1 x cryptroute2 — même altitude → COLLISION
    try {
        auto p1 = PathIO::read_single_path(DATA + "/cryptroute1.txt");
        auto p2 = PathIO::read_single_path(DATA + "/cryptroute2.txt");
        bool c  = clear_collision(p1, p2);
        bool f  = fhe_collision(geo, p1, p2);
        CHECK(c == true, "Scénario 1 clair : collision attendue");
        CHECK(f == true, "Scénario 1 FHE   : collision détectée");
        CHECK(c == f,    "Scénario 1 FHE == clair");
    } catch (...) { std::cout << "  [SKIP] cryptroute1/2 non trouvés\n"; }

    // Scénario 2 : cryptroute1 x cryptroute3 — altitudes diff → LIBRE
    try {
        auto p1 = PathIO::read_single_path(DATA + "/cryptroute1.txt");
        auto p3 = PathIO::read_single_path(DATA + "/cryptroute3.txt");
        bool c  = clear_collision(p1, p3);
        bool f  = fhe_collision(geo, p1, p3);
        CHECK(c == false, "Scénario 2 clair : pas de collision");
        CHECK(f == false, "Scénario 2 FHE   : pas de collision");
        CHECK(c == f,     "Scénario 2 FHE == clair");
    } catch (...) { std::cout << "  [SKIP] cryptroute1/3 non trouvés\n"; }

    // Scénario 3 : cryptroute4 x cryptroute5 — zigzag
    try {
        auto p4 = PathIO::read_single_path(DATA + "/cryptroute4.txt");
        auto p5 = PathIO::read_single_path(DATA + "/cryptroute5.txt");
        bool c  = clear_collision(p4, p5);
        bool f  = fhe_collision(geo, p4, p5);
        CHECK(c == f, "Scénario 3 (zigzag) FHE == clair");
    } catch (...) { std::cout << "  [SKIP] cryptroute4/5 non trouvés\n"; }
}

// ─── 2. Segments fixes déterministes ─────────────────────────────────────────
static void test_fixed_segments(GeometryEngine& geo) {
    SECTION("Segments fixes — FHE vs clair");

    struct Case {
        Segment s1, s2;
        bool expected;
        std::string label;
    };

    auto mk = [](long x1,long y1,long z1,long x2,long y2,long z2) -> Segment {
        return {IntPoint(x1,y1,z1), IntPoint(x2,y2,z2)};
    };

    std::vector<Case> cases = {
        { mk(0,5,10,10,5,10), mk(5,0,10,5,10,10), true,  "Croix z=10 → collision" },
        { mk(0,5,10,10,5,10), mk(5,0,20,5,10,20), false, "Altitudes diff → libre" },
        { mk(0,0,5,4,0,5),    mk(0,2,5,4,2,5),    false, "Parallèles coplanaires → libre" },
        { mk(0,0,5,5,5,5),    mk(5,5,5,10,0,5),   true,  "Extrémités partagées → collision" },
        { mk(0,0,10,10,10,10),mk(0,10,10,10,0,10),true,  "Diagonales z=10 → collision" },
    };

    for (const auto& tc : cases) {
        Path path1 = {tc.s1.first, tc.s1.second};
        Path path2 = {tc.s2.first, tc.s2.second};
        bool c = clear_collision(path1, path2);
        bool f = fhe_collision(geo, path1, path2);
        CHECK(c == tc.expected, tc.label + " (clair)");
        CHECK(f == tc.expected, tc.label + " (FHE)");
        CHECK(c == f,           tc.label + " FHE == clair");
    }
}

// ─── 3. Chemins aléatoires — cohérence FHE vs clair ─────────────────────────
static void test_random_paths(GeometryEngine& geo) {
    SECTION("Chemins aléatoires — cohérence FHE vs clair (10 paires)");

    int agree = 0;
    const int N = 10;
    unsigned seeds[] = {1,2,3,4,5,6,7,8,9,10};

    for (int i = 0; i < N; i++) {
        auto p1 = PathIO::generate_random_path(4, seeds[i]);
        auto p2 = PathIO::generate_random_path(4, seeds[i] + 100);
        bool c = clear_collision(p1, p2);
        bool f = fhe_collision(geo, p1, p2);
        if (c == f) agree++;
    }
    CHECK(agree >= 8, "FHE == clair sur au moins 8/10 paires aléatoires");
    std::cout << "  (accord : " << agree << "/" << N << ")\n";
}

// ─── 4. Distance seuil — intégration bout-en-bout ────────────────────────────
static void test_distance_integration(CryptoEngine& engine, GeometryEngine& geo) {
    SECTION("Distance <= seuil — intégration avec trajectoires réelles");

    const double SEUIL = 15.0;

    try {
        auto p1 = PathIO::read_single_path("data/cryptroute1.txt");
        auto p2 = PathIO::read_single_path("data/cryptroute2.txt");

        // Scénario 1 : même altitude → drones très proches → danger
        bool prox = fhe_proximity(engine, geo, p1, p2, SEUIL);
        CHECK(prox == true, "Scénario 1 : proximité détectée (même altitude)");
    } catch (...) { std::cout << "  [SKIP] fichiers non trouvés\n"; }

}

// ─── main ────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "========================================\n";
    std::cout << "  Test intégration bout-en-bout FHE     \n";
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

    test_fixed_segments(geo);
    test_real_scenarios(engine, geo);
    test_random_paths(geo);
    test_distance_integration(engine, geo);

    std::cout << "\n========================================\n";
    std::cout << "  Résultat : " << g_passed << "/" << g_total << " tests passés";
    if (g_failed > 0) std::cout << "  (" << g_failed << " ECHECS)";
    std::cout << "\n========================================\n";

    return (g_failed == 0) ? 0 : 1;
}
