/**
 * test_fhe_validation.cpp
 * ===================
 * Test de cohérence FHE vs clair.
 *
 * Pour chaque paire de segments tirés aléatoirement :
 *   - on calcule le résultat EN CLAIR  (doSegmentsIntersectClear3D)
 *   - on calcule le résultat EN FHE    (batchCheckIntersection3D)
 *   - on vérifie qu'ils sont IDENTIQUES
 *
 * Cela valide que le protocole chiffré ne produit pas de faux positifs
 * ni de faux négatifs par rapport à la référence en clair.
 *
 * Compile : cmake --build build --target test_fhe_validation
 * Run     : ./build/test_fhe_validation [nb_paires]   (défaut : 20)
 *
 * AVERTISSEMENT : l'initialisation FHE prend ~60 s (keygen + scheme switching).
 * Le test lui-même (N paires) est rapide ensuite.
 */

#include "engine.hpp"
#include "geometry.hpp"
#include "io.hpp"
#include "types.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

// ─── mini framework ──────────────────────────────────────────────────────────
static int g_total  = 0;
static int g_passed = 0;
static int g_failed = 0;
static int g_known  = 0;   // limitations connues : documentées, pas comptées comme échecs

static void CHECK(bool cond, const std::string &label)
{
    ++g_total;
    if (cond) { ++g_passed; std::cout << "  [PASS] " << label << "\n"; }
    else       { ++g_failed; std::cout << "  [FAIL] " << label << "\n"; }
}

// Limitation connue du protocole FHE : on la signale mais elle ne fait pas
// échouer le test, et elle n'est pas comptée dans g_total/g_failed.
static void CHECK_KNOWN_LIMIT(bool cond, const std::string &label,
                               const std::string &reason)
{
    ++g_known;
    if (cond)
        std::cout << "  [PASS] " << label << "  (cas connu résolu)\n";
    else
        std::cout << "  [LIMIT] " << label
                  << "\n           -> LIMITATION CONNUE : " << reason << "\n";
}

// ─── générateur de segments aléatoires ───────────────────────────────────────
static Segment random_segment(std::mt19937 &rng)
{
    std::uniform_int_distribution<long> dist(
        DroneConstants::MIN_COORDINATE, DroneConstants::MAX_COORDINATE);

    IntPoint a, b;
    do {
        a = IntPoint(dist(rng), dist(rng), dist(rng));
        b = IntPoint(dist(rng), dist(rng), dist(rng));
    } while (a == b); // garantit que les deux extrémités sont distinctes

    return {a, b};
}

// ─── scénarios fixes (déterministes) ─────────────────────────────────────────
struct FixedCase
{
    Segment s1, s2;
    bool    expected;
    std::string label;
};

static std::vector<FixedCase> fixed_cases()
{
    auto mk = [](long x1,long y1,long z1,long x2,long y2,long z2) -> Segment {
        return {IntPoint(x1,y1,z1), IntPoint(x2,y2,z2)};
    };

    return {
        // collision certaine
        { mk(0,5,10,10,5,10), mk(5,0,10,5,10,10), true,
          "Scénario 1 : croix dans z=10" },
        // altitudes différentes
        { mk(0,5,10,10,5,10), mk(5,0,20,5,10,20), false,
          "Scénario 2 : altitudes z=10 vs z=20" },
        // parallèles coplanaires disjoints
        { mk(0,0,5,4,0,5), mk(0,2,5,4,2,5), false,
          "Parallèles coplanaires disjoints" },
        // extrémités partagées
        { mk(0,0,5,5,5,5), mk(5,5,5,10,0,5), true,
          "Extrémités partagées en 3D" },
        // LIMITATION : segments identiques et diagonaux (z varie le long du segment).
        // Dans batchCheckIntersection3D, le code parallèle vérifie
        // p1.z != q1.z pour détecter les segments non horizontaux et leur attribue
        // cops=9999 (non coplanaire) → faux négatif.
        // On utilise CHECK_KNOWN_LIMIT pour ce cas (voir run_fixed_validation).
        { mk(0,0,0,10,10,10), mk(0,0,0,10,10,10), true,
          "Segments identiques (superposés, diagonaux)" },
        // diagonales scénario 1
        { mk(0,0,10,10,10,10), mk(0,10,10,10,0,10), true,
          "Diagonales plan z=10" },
        // diagonales scénario 2 (altitudes diff.)
        { mk(0,0,10,10,10,10), mk(0,10,20,10,0,20), false,
          "Diagonales altitudes diff." },
    };
}

// ─── test validation aléatoire ───────────────────────────────────────────────────
static void run_random_validation(GeometryEngine &geo,
                               int N, unsigned seed)
{
    std::cout << "\n=== Validation aléatoire : " << N
              << " paires (seed=" << seed << ") ===\n";

    std::mt19937 rng(seed);

    int agree = 0, disagree = 0;
    int fp = 0, fn = 0;         // faux positifs / faux négatifs

    for (int i = 0; i < N; i++)
    {
        Segment s1 = random_segment(rng);
        Segment s2 = random_segment(rng);

        // Référence en clair
        bool clear_result = GeometryEngine::doSegmentsIntersectClear3D(s1, s2);

        // Résultat FHE (batching : on met s2 dans un vecteur de taille 1)
        std::vector<Segment> neighbors = {s2};
        auto fhe_vec = geo.batchCheckIntersection3D(s1, neighbors);
        bool fhe_result = (fhe_vec[0] > 0.5);

        if (clear_result == fhe_result)
            ++agree;
        else
        {
            ++disagree;
            if (fhe_result && !clear_result) ++fp;
            if (!fhe_result && clear_result) ++fn;

            // Afficher le détail des désaccords
            std::cout << "  [DESACCORD #" << (i+1) << "] "
                      << "clair=" << clear_result
                      << "  fhe="  << fhe_result
                      << "  fhe_val=" << std::fixed << std::setprecision(4)
                      << fhe_vec[0] << "\n"
                      << "    s1: (" << s1.first.x  << "," << s1.first.y  << ","
                                     << s1.first.z  << ") -> ("
                                     << s1.second.x << "," << s1.second.y << ","
                                     << s1.second.z << ")\n"
                      << "    s2: (" << s2.first.x  << "," << s2.first.y  << ","
                                     << s2.first.z  << ") -> ("
                                     << s2.second.x << "," << s2.second.y << ","
                                     << s2.second.z << ")\n";
        }
    }

    double rate = 100.0 * agree / N;
    std::cout << "\n  Accord clair/FHE     : " << agree << "/" << N
              << " (" << std::fixed << std::setprecision(1) << rate << "%)\n";
    std::cout << "  Faux positifs FHE    : " << fp << "\n";
    std::cout << "  Faux négatifs FHE    : " << fn << "\n";

    CHECK(disagree == 0,
          "Cohérence totale FHE == clair sur " + std::to_string(N) + " paires aléatoires");
}

// ─── test validation sur scénarios fixes ─────────────────────────────────────────
static void run_fixed_validation(GeometryEngine &geo)
{
    std::cout << "\n=== Validation scénarios fixes ===\n";

    for (const auto &tc : fixed_cases())
    {
        std::vector<Segment> neighbors = {tc.s2};
        auto fhe_vec = geo.batchCheckIntersection3D(tc.s1, neighbors);
        bool fhe_result = (fhe_vec[0] > 0.5);

        std::string label = tc.label
            + "  [attendu=" + (tc.expected ? "OUI" : "NON")
            + "  FHE="     + (fhe_result  ? "OUI" : "NON")
            + "  val="     + std::to_string(fhe_vec[0]).substr(0,6) + "]";

        // Le cas "segments identiques diagonaux" est une limitation connue du
        // protocole FHE : le code parallèle de batchCheckIntersection3D n'est
        // conçu que pour des segments à altitude constante (z fixe).
        // Cas colineaires : ancienne limitation corrigee.
        // On utilise desormais un test d'overlap FHE (4 SS TFHE) au lieu
        // du test d'orientation 2D (qui donnait des valeurs = 0 non comparables).
        CHECK(fhe_result == tc.expected, label);
    }
}

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    int N = 20;
    if (argc > 1) N = std::atoi(argv[1]);
    if (N < 1)    N = 1;

    std::cout << "========================================\n";
    std::cout << "  Test validation FHE vs clair              \n";
    std::cout << "========================================\n";

    // ── Initialisation du moteur FHE ────────────────────────────────────────
    std::cout << "\n[*] Initialisation CryptoEngine...\n";
    auto t0 = std::chrono::steady_clock::now();

    CryptoEngine engine;
    CryptoEngine::Config cfg;
    cfg.batchSize    = 64;
    cfg.switchValues = 64;
    cfg.logQ_ccLWE   = 25;
    engine.initialize(cfg);

    std::cout << "[*] Setup scheme switching...\n";
    engine.setupSchemeSwitching();

    auto t1 = std::chrono::steady_clock::now();
    double init_s = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "[*] Initialisation terminée en "
              << std::fixed << std::setprecision(1) << init_s << " s\n";

    GeometryEngine geo(&engine);

    // ── Tests ────────────────────────────────────────────────────────────────
    run_fixed_validation(geo);
    run_random_validation(geo, N, /*seed=*/42);

    // ── Résumé ───────────────────────────────────────────────────────────────
    std::cout << "\n========================================\n";
    std::cout << "  Résultat : " << g_passed << "/" << g_total << " tests passés";
    if (g_failed > 0)
        std::cout << "  (" << g_failed << " ECHECS)";
    if (g_known > 0)
        std::cout << "\n  Limitations connues documentées : " << g_known;
    std::cout << "\n========================================\n";

    return (g_failed == 0) ? 0 : 1;
}
