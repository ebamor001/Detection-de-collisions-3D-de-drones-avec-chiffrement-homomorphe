/**
 * main.cpp — Banc de test complet
 * Drone Path Intersection Detection via FHE (OpenFHE / CKKS + FHEW)
 *
 * Tests couverts :
 *   1.  Initialisation du CryptoEngine
 *   2.  Setup du scheme switching
 *   3.  Opérations arithmétiques chiffrées de base
 *   4.  Comparaisons chiffrées (GT, GE, LE, near-zero)
 *   5.  Calcul d'orientation chiffré vs. clair
 *   6.  Détection d'intersection — cas généraux
 *   7.  Détection d'intersection — cas colinéaires / endpoints
 *   8.  *** Chargement et test sur les fichiers cryptroute*.txt ***
 *   9.  *** Test sur multiple_routes.txt (toutes paires) ***
 *  10.  Parsing et validation de fichiers (PathIO)
 *  11.  Benchmark : temps moyen par test d'intersection
 */

#include "engine.hpp"
#include "geometry.hpp"
#include "intersection.hpp"
#include "io.hpp"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────
// Utilitaires d'affichage
// ─────────────────────────────────────────────────────────────────────────────

static int g_passed = 0;
static int g_failed = 0;

static void printSeparator(const std::string& title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}

static void checkResult(const std::string& testName, bool expected, double decrypted,
                        double threshold = 0.5) {
    bool got = (decrypted > threshold);
    bool ok  = (got == expected);
    if (ok) {
        std::cout << "  [PASS] " << testName
                  << "  (val=" << std::fixed << std::setprecision(4) << decrypted << ")\n";
        ++g_passed;
    } else {
        std::cout << "  [FAIL] " << testName
                  << "  expected=" << (expected ? "1" : "0")
                  << "  got="      << (got      ? "1" : "0")
                  << "  val="      << std::fixed << std::setprecision(4) << decrypted << "\n";
        ++g_failed;
    }
}

static void checkApprox(const std::string& testName, double expected, double got,
                        double tol = 0.05) {
    bool ok = std::abs(got - expected) <= tol;
    if (ok) {
        std::cout << "  [PASS] " << testName
                  << "  (expected≈" << expected << "  got="
                  << std::fixed << std::setprecision(4) << got << ")\n";
        ++g_passed;
    } else {
        std::cout << "  [FAIL] " << testName
                  << "  expected≈" << expected << "  got="
                  << std::fixed << std::setprecision(4) << got << "\n";
        ++g_failed;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Résolution automatique du dossier data/
// ─────────────────────────────────────────────────────────────────────────────

static std::string findDataDir(const std::string& argv0) {
    fs::path base = fs::canonical(fs::path(argv0)).parent_path();
    for (int i = 0; i < 4; ++i) {
        fs::path candidate = base / "data";
        if (fs::is_directory(candidate)) return candidate.string();
        base = base.parent_path();
    }
    return "data"; // fallback relatif
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Initialisation
// ─────────────────────────────────────────────────────────────────────────────

static CryptoEngine* initEngine() {
    printSeparator("1. Initialisation CryptoEngine");

    CryptoEngine::Config cfg;
    cfg.multDepth    = 17;
    cfg.scaleModSize = 50;
    cfg.batchSize    = 1;
    cfg.ringDim      = 8192;
    cfg.scaleSign    = 512.0;
    cfg.guardGain    = 1.0;
    cfg.logQ_ccLWE   = 25;

    auto* engine = new CryptoEngine();
    engine->initialize(cfg);
    std::cout << "  [OK] CryptoEngine initialisé\n";
    return engine;
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Scheme switching
// ─────────────────────────────────────────────────────────────────────────────

static void testSchemeSwitching(CryptoEngine& engine) {
    printSeparator("2. Setup Scheme Switching");
    engine.setupSchemeSwitching();
    std::cout << "  [OK] Scheme switching prêt\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Arithmétique chiffrée de base
// ─────────────────────────────────────────────────────────────────────────────

static void testArithmetic(CryptoEngine& engine) {
    printSeparator("3. Arithmétique chiffrée");

    auto ct3 = engine.encryptValue(3.0);
    auto ct7 = engine.encryptValue(7.0);

    checkApprox("3 + 7 = 10",  10.0, engine.decryptValue(engine.add(ct3, ct7)));
    checkApprox("7 - 3 = 4",    4.0, engine.decryptValue(engine.sub(ct7, ct3)));
    checkApprox("3 * 7 = 21",  21.0, engine.decryptValue(engine.mult(ct3, ct7)));
    checkApprox("neg(3) = -3", -3.0, engine.decryptValue(engine.negate(ct3)));
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Comparaisons chiffrées
// ─────────────────────────────────────────────────────────────────────────────

static void testComparisons(CryptoEngine& engine) {
    printSeparator("4. Comparaisons chiffrées");

    checkResult("GTZ( 5) = 1", true,
        engine.decryptValue(engine.compareGreaterThanZero(engine.encryptValue( 5.0))));
    checkResult("GTZ(-5) = 0", false,
        engine.decryptValue(engine.compareGreaterThanZero(engine.encryptValue(-5.0))));

    auto a = engine.encryptValue(8.0);
    auto b = engine.encryptValue(3.0);
    checkResult("GT(8,3) = 1", true,  engine.decryptValue(engine.compareGT(a, b)));
    checkResult("GT(3,8) = 0", false, engine.decryptValue(engine.compareGT(b, a)));

    auto c = engine.encryptValue(5.0);
    auto d = engine.encryptValue(5.0);
    checkResult("GE(5,5) = 1", true,  engine.decryptValue(engine.compareGE(c, d)));
    checkResult("LE(3,8) = 1", true,  engine.decryptValue(engine.compareLE(b, a)));

    checkResult("ltZero(-4) = 1", true,
        engine.decryptValue(engine.ltZero(engine.encryptValue(-4.0))));
    checkResult("nearZero(0.001, tau=0.01) = 1", true,
        engine.decryptValue(engine.isNearZeroBand(engine.encryptValue(0.001), 0.01)));
    checkResult("nearZero(1.0,   tau=0.01) = 0", false,
        engine.decryptValue(engine.isNearZeroBand(engine.encryptValue(1.0),   0.01)));
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Orientation chiffrée vs. clair
// ─────────────────────────────────────────────────────────────────────────────

static void testOrientation(CryptoEngine& engine, GeometryEngine& geo) {
    printSeparator("5. Orientation chiffrée vs. clair");

    IntPoint p0{0,0}, q0{1,0}, r0{0,1};
    checkApprox("CCW ~ -1", -1.0,
        engine.decryptValue(geo.computeOrientationValue(p0, q0, r0)), 0.1);

    IntPoint p1{0,0}, q1{0,1}, r1{1,0};
    checkApprox("CW  ~ +1",  1.0,
        engine.decryptValue(geo.computeOrientationValue(p1, q1, r1)), 0.1);

    IntPoint p2{0,0}, q2{1,1}, r2{2,2};
    checkApprox("COL ~  0",  0.0,
        engine.decryptValue(geo.computeOrientationValue(p2, q2, r2)), 0.1);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Cas généraux
// ─────────────────────────────────────────────────────────────────────────────

static void testIntersectionGeneral(CryptoEngine& engine, GeometryEngine& geo) {
    printSeparator("6. Intersections — cas généraux");

    struct TC { std::string name; IntPoint p1,q1,p2,q2; bool expected; };
    std::vector<TC> cases = {
        {"Croix simple",         {0,1},{2,1},{1,0},{1,2}, true },
        {"Parallèles horiz",     {0,0},{4,0},{0,1},{4,1}, false},
        {"Colinéaires disjoints",{0,0},{1,0},{2,0},{3,0}, false},
        {"Endpoint commun",      {0,0},{2,2},{2,2},{4,0}, true },
        {"T-intersection",       {0,0},{4,0},{2,-1},{2,1},true },
        {"Oblique",              {0,0},{3,3},{0,3},{3,0}, true },
        {"Adjacents",            {0,0},{1,0},{2,0},{3,0}, false},
    };

    for (auto& tc : cases) {
        Segment s1 = {{tc.p1},{tc.q1}};
        Segment s2 = {{tc.p2},{tc.q2}};
        checkResult(tc.name, tc.expected,
                    engine.decryptValue(geo.checkSegmentIntersection(s1, s2)));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. Cas colinéairesS
// ─────────────────────────────────────────────────────────────────────────────

static void testIntersectionCollinear(CryptoEngine& engine, GeometryEngine& geo) {
    printSeparator("7. Intersections — colinéaires & endpoints");

    struct TC { std::string name; IntPoint p1,q1,p2,q2; bool expected; };
    std::vector<TC> cases = {
        {"Overlap colinéaire",    {0,0},{3,0},{1,0},{4,0}, true },
        {"Touch colinéaire",      {0,0},{2,0},{2,0},{4,0}, true },
        {"Disjoints colinéaires", {0,0},{1,0},{3,0},{4,0}, false},
        {"Point sur segment",     {0,1},{2,1},{1,0},{1,2}, true },
    };

    for (auto& tc : cases) {
        Segment s1 = {{tc.p1},{tc.q1}};
        Segment s2 = {{tc.p2},{tc.q2}};
        checkResult(tc.name, tc.expected,
                    engine.decryptValue(geo.checkSegmentIntersection(s1, s2)));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. cryptroute1.txt … cryptroute5.txt
// ─────────────────────────────────────────────────────────────────────────────

static void testCryptoRoutes(CryptoEngine& engine, GeometryEngine& geo,
                             const std::string& dataDir) {
    printSeparator("8. cryptroute*.txt — toutes paires");

    std::vector<std::pair<std::string, Path>> routes;
    for (int i = 1; i <= 5; ++i) {
        std::string filepath = dataDir + "/cryptroute" + std::to_string(i) + ".txt";
        if (!fs::exists(filepath)) {
            std::cout << "  [SKIP] cryptroute" << i << ".txt introuvable\n";
            continue;
        }
        try {
            Path p = PathIO::read_single_path(filepath);
            routes.push_back({"cryptroute" + std::to_string(i), p});
            PathIO::print_path(p, "  " + routes.back().first);
        } catch (const std::exception& e) {
            std::cout << "  [ERREUR] cryptroute" << i << " : " << e.what() << "\n";
        }
    }

    if (routes.size() < 2) {
        std::cout << "  Moins de 2 routes chargées — test ignoré.\n";
        return;
    }

    IntersectionDetector detector(&engine, &geo);

    for (size_t i = 0; i < routes.size(); ++i) {
        for (size_t j = i + 1; j < routes.size(); ++j) {
            const auto& [ni, pi] = routes[i];
            const auto& [nj, pj] = routes[j];
            std::cout << "\n  ── " << ni << " x " << nj << " ──\n";

            // Référence en clair
            auto clearRes = detector.detectIntersectionsClear(pi, pj);
            int clearCount = 0;
            for (auto& r : clearRes) if (r.intersects) ++clearCount;

            // Résultat chiffré (déchiffrement du résultat final seulement)
            auto encRes = detector.detectIntersectionsHybrid(pi, pj);
            int encCount = 0;
            for (auto& r : encRes) if (r.intersects) ++encCount;

            std::cout << "  Clair   : " << clearCount << " intersection(s)\n";
            std::cout << "  Chiffré : " << encCount   << " intersection(s)\n";

            if (clearCount == encCount) {
                ++g_passed;
                std::cout << "  [PASS] Cohérent\n";
            } else {
                ++g_failed;
                std::cout << "  [FAIL] Divergence clair/chiffré !\n";
            }
        }
    }
    detector.printStatistics();
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. multiple_routes.txt
// ─────────────────────────────────────────────────────────────────────────────

static void testMultipleRoutes(CryptoEngine& engine, GeometryEngine& geo,
                               const std::string& dataDir) {
    printSeparator("9. multiple_routes.txt — toutes paires");

    std::string filepath = dataDir + "/multiple_routes.txt";
    if (!fs::exists(filepath)) {
        std::cout << "  [SKIP] multiple_routes.txt introuvable\n";
        return;
    }

    std::vector<Path> routes;
    try {
        routes = PathIO::read_multiple_paths(filepath);
    } catch (const std::exception& e) {
        std::cout << "  [ERREUR] " << e.what() << "\n";
        return;
    }

    std::cout << "  " << routes.size() << " route(s) chargée(s)\n";
    for (size_t i = 0; i < routes.size(); ++i)
        PathIO::print_path(routes[i], "  Route " + std::to_string(i + 1));

    if (routes.size() < 2) {
        std::cout << "  Pas assez de routes.\n";
        return;
    }

    IntersectionDetector detector(&engine, &geo);
    int totalPairs = 0, collisionPairs = 0;

    for (size_t i = 0; i < routes.size(); ++i) {
        for (size_t j = i + 1; j < routes.size(); ++j) {
            std::cout << "\n  ── Route " << (i+1) << " x Route " << (j+1) << " ──\n";

            auto clearRes = detector.detectIntersectionsClear(routes[i], routes[j]);
            int clearCount = 0;
            for (auto& r : clearRes) if (r.intersects) ++clearCount;

            auto encRes = detector.detectIntersectionsHybrid(routes[i], routes[j]);
            int encCount = 0;
            for (auto& r : encRes) if (r.intersects) ++encCount;

            std::cout << "  Clair   : " << clearCount << " intersection(s)\n";
            std::cout << "  Chiffré : " << encCount   << " intersection(s)\n";

            if (clearCount == encCount) {
                ++g_passed; std::cout << "  [PASS]\n";
            } else {
                ++g_failed; std::cout << "  [FAIL] Divergence !\n";
            }

            if (encCount > 0) ++collisionPairs;
            ++totalPairs;
        }
    }

    std::cout << "\n  Bilan : " << collisionPairs
              << " paire(s) en collision / " << totalPairs << " testée(s)\n";
    detector.printStatistics();
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. Parsing PathIO
// ─────────────────────────────────────────────────────────────────────────────

static void testPathIO() {
    printSeparator("10. PathIO — parsing & validation");

    // Format exact de tes fichiers
    Path p = PathIO::parse_line("(0,0)(50,50)(99,0)");
    bool ok = (p.size() == 3 &&
               p[0].x == 0  && p[0].y == 0  &&
               p[1].x == 50 && p[1].y == 50 &&
               p[2].x == 99 && p[2].y == 0);
    if (ok) { ++g_passed; std::cout << "  [PASS] parse_line format cryptoroute\n"; }
    else    { ++g_failed; std::cout << "  [FAIL] parse_line format cryptoroute\n"; }

    // Négatifs
    Path p2 = PathIO::parse_line("(-10,-20)(0,0)(30,-5)");
    bool ok2 = (p2.size() == 3 && p2[0].x == -10 && p2[0].y == -20);
    if (ok2) { ++g_passed; std::cout << "  [PASS] parse_line négatifs\n"; }
    else     { ++g_failed; std::cout << "  [FAIL] parse_line négatifs\n"; }

    // Validation
    Path valid = {{0,0},{50,50},{99,0}};
    Path court = {{0,0}};
    Path dup   = {{0,0},{0,0},{1,1}};

    if ( PathIO::validate_path(valid)) { ++g_passed; std::cout << "  [PASS] validate OK\n"; }
    else                               { ++g_failed; std::cout << "  [FAIL] validate OK\n"; }
    if (!PathIO::validate_path(court)) { ++g_passed; std::cout << "  [PASS] validate trop court\n"; }
    else                               { ++g_failed; std::cout << "  [FAIL] validate trop court\n"; }
    if (!PathIO::validate_path(dup))   { ++g_passed; std::cout << "  [PASS] validate doublons\n"; }
    else                               { ++g_failed; std::cout << "  [FAIL] validate doublons\n"; }

    // Aller-retour fichier
    const std::string tmp = "/tmp/test_route.txt";
    PathIO::write_path(valid, tmp);
    try {
        Path reload = PathIO::read_single_path(tmp);
        bool ioOk = (reload.size() == valid.size());
        for (size_t i = 0; i < valid.size() && ioOk; ++i)
            ioOk = (reload[i] == valid[i]);
        if (ioOk) { ++g_passed; std::cout << "  [PASS] write + read aller-retour\n"; }
        else      { ++g_failed; std::cout << "  [FAIL] write + read aller-retour\n"; }
    } catch (const std::exception& e) {
        ++g_failed;
        std::cout << "  [FAIL] " << e.what() << "\n";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 11. Benchmark sur les vraies cryptoroutes
// ─────────────────────────────────────────────────────────────────────────────

static void benchmark(CryptoEngine& engine, GeometryEngine& geo,
                      const std::string& dataDir) {
    printSeparator("11. Benchmark — cryptoroute1 x cryptoroute2");

    std::string f1 = dataDir + "/cryptroute1.txt";
    std::string f2 = dataDir + "/cryptroute2.txt";
    if (!fs::exists(f1) || !fs::exists(f2)) {
        std::cout << "  [SKIP] fichiers introuvables\n";
        return;
    }

    Path p1 = PathIO::read_single_path(f1);
    Path p2 = PathIO::read_single_path(f2);
    int  nb = (int)((p1.size()-1) * (p2.size()-1));

    IntersectionDetector detector(&engine, &geo);
    detector.resetStatistics();

    auto t0 = std::chrono::high_resolution_clock::now();
    detector.detectIntersectionsHybrid(p1, p2);
    auto t1 = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "  " << nb << " paire(s) de segments testée(s)\n";
    std::cout << "  Temps total  : " << std::fixed << std::setprecision(1) << ms << " ms\n";
    if (nb > 0)
        std::cout << "  Temps/paire  : " << std::fixed << std::setprecision(1)
                  << ms / nb << " ms\n";
    geo.printStats();
}

// ─────────────────────────────────────────────────────────────────────────────
// Résumé final
// ─────────────────────────────────────────────────────────────────────────────

static void printSummary() {
    printSeparator("Résumé global");
    int total = g_passed + g_failed;
    std::cout << "  Tests passés : " << g_passed << " / " << total << "\n";
    if (g_failed == 0)
        std::cout << "  ✓ Tous les tests sont OK\n";
    else
        std::cout << "  ✗ " << g_failed << " test(s) en échec\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║   Drone FHE — Détection de collisions chiffrée          ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";

    bool        skipBench = false;
    std::string dataDir   = findDataDir(argv[0]);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--no-bench")              skipBench = true;
        if (arg == "--data" && i+1 < argc)    dataDir   = argv[++i];
    }

    std::cout << "  Dossier data : " << dataDir << "\n";

    try {
        CryptoEngine* engine = initEngine();
        testSchemeSwitching(*engine);

        GeometryEngine geo(engine);

        // Tests unitaires
        testArithmetic(*engine);
        testComparisons(*engine);
        testOrientation(*engine, geo);
        testIntersectionGeneral(*engine, geo);
        testIntersectionCollinear(*engine, geo);

        // Tests sur les vrais fichiers
        testCryptoRoutes(*engine, geo, dataDir);
        testMultipleRoutes(*engine, geo, dataDir);

        // PathIO
        testPathIO();

        // Benchmark
        if (!skipBench)
            benchmark(*engine, geo, dataDir);

        delete engine;

    } catch (const std::exception& e) {
        std::cerr << "\n[EXCEPTION] " << e.what() << "\n";
        ++g_failed;
    }

    printSummary();
    return (g_failed == 0) ? 0 : 1;
}
