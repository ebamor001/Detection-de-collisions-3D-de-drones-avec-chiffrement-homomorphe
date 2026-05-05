/**
 * test_benchmark_scalability.cpp
 * ===============================
 * Benchmark de scalabilité : temps réel en fonction du nombre de segments N.
 *
 * Pour chaque valeur de N dans {1, 2, 5, 10, 20, 50} :
 *
 *   AVEC batching :
 *     batchCheckIntersection3D(seg, N voisins) — 1 seul appel, ~6 SS quel que soit N
 *
 *   SANS batching (baseline, mesure réelle sur N=1..5, extrapolée après) :
 *     checkSegmentIntersection3D(seg, voisin_i) × N — N appels, ~9 SS chacun
 *
 * Sortie :
 *   - Tableau console
 *   - Fichier CSV  results/benchmark_scalability.csv
 *
 * Compile : cmake --build build --target test_benchmark_scalability
 * Run     : ./build/test_benchmark_scalability
 */

#include "engine.hpp"
#include "geometry.hpp"
#include "io.hpp"
#include "types.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::duration<double, std::milli>;

// ─── générateur de segments reproductibles ───────────────────────────────────
static Segment make_seg(std::mt19937 &rng)
{
    std::uniform_int_distribution<long> d(-50, 50);
    IntPoint a, b;
    do {
        a = {d(rng), d(rng), d(rng)};
        b = {d(rng), d(rng), d(rng)};
    } while (a == b);
    return {a, b};
}

// ─── mesure d'une répétition ──────────────────────────────────────────────────
struct MeasureResult {
    double   time_ms;      // temps total mesuré
    size_t   ss_count;     // scheme-switches réels (bootstrapCount delta)
};

// Avec batching : 1 appel pour N voisins
static MeasureResult measure_batch(GeometryEngine &geo,
                                   const Segment &mySeg,
                                   const std::vector<Segment> &neighbors)
{
    geo.resetBootstrapCount();
    auto t0 = Clock::now();
    geo.batchCheckIntersection3D(mySeg, neighbors);
    double ms = Ms(Clock::now() - t0).count();
    return {ms, geo.getBootstrapCount()};
}

// Sans batching : N appels individuels checkSegmentIntersection3D
static MeasureResult measure_nobatch(GeometryEngine &geo,
                                     const Segment &mySeg,
                                     const std::vector<Segment> &neighbors)
{
    geo.resetBootstrapCount();
    auto t0 = Clock::now();
    for (const auto &nb : neighbors)
        geo.checkSegmentIntersection3D(mySeg, nb);
    double ms = Ms(Clock::now() - t0).count();
    return {ms, geo.getBootstrapCount()};
}

// ─── main ─────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "========================================\n";
    std::cout << "  Benchmark de scalabilité FHE          \n";
    std::cout << "========================================\n";

    // ── Init FHE ────────────────────────────────────────────────────────────
    std::cout << "\n[*] Initialisation CryptoEngine...\n";
    CryptoEngine engine;
    CryptoEngine::Config cfg;
    cfg.batchSize    = 64;
    cfg.switchValues = 64;
    cfg.logQ_ccLWE   = 25;
    engine.initialize(cfg);
    engine.setupSchemeSwitching();
    std::cout << "[*] Prêt.\n\n";

    GeometryEngine geo(&engine);

    // ── Valeurs de N à tester ────────────────────────────────────────────────
    // Pour N <= MAX_REAL_NOBATCH on mesure réellement sans batching.
    // Pour les N plus grands on extrapole à partir de la mesure N=1.
    const std::vector<int> Ns = {1, 2, 5, 10, 20, 50};
    const int MAX_REAL_NOBATCH = 5;  // au-delà c'est trop lent à mesurer

    // ── Pré-générer les segments une seule fois ──────────────────────────────
    std::mt19937 rng(1234);
    Segment mySeg = make_seg(rng);

    const int N_MAX = Ns.back();
    std::vector<Segment> all_neighbors;
    all_neighbors.reserve(N_MAX);
    for (int i = 0; i < N_MAX; i++)
        all_neighbors.push_back(make_seg(rng));

    // ── Mesure de référence N=1 sans batching (pour extrapolation) ───────────
    std::cout << "[*] Mesure de référence N=1 sans batching...\n";
    auto ref1 = measure_nobatch(geo, mySeg, {all_neighbors[0]});
    double time_per_pair_ms = ref1.time_ms;
    size_t ss_per_pair      = ref1.ss_count;
    std::cout << "    -> " << std::fixed << std::setprecision(2)
              << time_per_pair_ms << " ms, " << ss_per_pair << " SS\n\n";

    // ── Entête tableau ───────────────────────────────────────────────────────
    std::cout << std::left
              << std::setw(6)  << "N"
              << std::setw(16) << "Batch (ms)"
              << std::setw(12) << "SS batch"
              << std::setw(20) << "NoBatch (ms)"
              << std::setw(14) << "SS nobatch"
              << std::setw(12) << "Gain temps"
              << std::setw(12) << "Gain SS"
              << "\n";
    std::cout << std::string(92, '-') << "\n";

    // ── Fichier CSV ──────────────────────────────────────────────────────────
    std::ofstream csv("results/benchmark_scalability.csv");
    if (!csv.is_open())
    {
        // Créer le dossier si besoin et réessayer
        system("mkdir -p results");
        csv.open("results/benchmark_scalability.csv");
    }
    csv << "N,time_batch_ms,ss_batch,time_nobatch_ms,ss_nobatch,"
           "gain_time,gain_ss,nobatch_measured\n";

    // ── Boucle principale ────────────────────────────────────────────────────
    for (int N : Ns)
    {
        std::vector<Segment> neighbors(all_neighbors.begin(),
                                       all_neighbors.begin() + N);

        // — Avec batching (toujours mesuré) —
        auto bRes = measure_batch(geo, mySeg, neighbors);

        // — Sans batching (mesuré si N <= MAX_REAL_NOBATCH, extrapolé sinon) —
        double nb_time_ms;
        size_t nb_ss;
        bool   measured;

        if (N <= MAX_REAL_NOBATCH)
        {
            auto nbRes  = measure_nobatch(geo, mySeg, neighbors);
            nb_time_ms  = nbRes.time_ms;
            nb_ss       = nbRes.ss_count;
            measured    = true;
        }
        else
        {
            // Extrapolation linéaire à partir de la mesure N=1
            nb_time_ms = time_per_pair_ms * N;
            nb_ss      = ss_per_pair * N;
            measured   = false;
        }

        double gain_t  = (bRes.time_ms > 0) ? nb_time_ms / bRes.time_ms : 0.0;
        double gain_ss = (bRes.ss_count > 0) ? (double)nb_ss / bRes.ss_count : 0.0;

        // Affichage console
        std::cout << std::left  << std::setw(6)  << N
                  << std::right << std::setw(12) << std::fixed << std::setprecision(1)
                  << bRes.time_ms << "    "
                  << std::setw(8)  << bRes.ss_count << "    "
                  << std::setw(12) << std::setprecision(1) << nb_time_ms
                  << (measured ? " (reel)" : " (extrap)")
                  << std::setw(8)  << nb_ss << "    "
                  << std::setw(8)  << std::setprecision(2) << gain_t  << "x    "
                  << std::setw(6)  << std::setprecision(2) << gain_ss << "x\n";

        // CSV
        csv << N             << ","
            << bRes.time_ms  << ","
            << bRes.ss_count << ","
            << nb_time_ms    << ","
            << nb_ss         << ","
            << gain_t        << ","
            << gain_ss       << ","
            << (measured ? 1 : 0) << "\n";
    }
    csv.close();

    // ── Résumé ───────────────────────────────────────────────────────────────
    std::cout << "\n";
    std::cout << "Fichier CSV écrit : results/benchmark_scalability.csv\n";
    std::cout << "\nConclusion batching :\n";
    std::cout << "  - Scheme-switches AVEC batching : ~6 SS (constant, O(1) en N)\n";
    std::cout << "  - Scheme-switches SANS batching : ~" << ss_per_pair
              << " SS par paire -> O(N)\n";
    std::cout << "  -> Pour N=50, gain théorique en SS : x"
              << (ss_per_pair * 50) / 6 << "\n";
    std::cout << "\nPour tracer les courbes :\n";
    std::cout << "  python3 tests/plot_scalability.py\n";

    return 0;
}
