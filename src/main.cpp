/**
 * main.cpp — Detection collision 3D FHE avec batching
 * Utilise batchCheckIntersection3D : 6 SS pour toutes les paires
 *
 * Usage : ./drone_fhe --path1 f1.txt --path2 f2.txt [--path3 f3.txt]
 * Format : (x,y,z)(x,y,z)...
 * Sortie : JSON_RESULT:{...}
 */

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <iomanip>
#include "geometry.hpp"
#include "engine.hpp"
#include "types.hpp"
#include "io.hpp"
#include "intersection.hpp"

static std::map<std::string, std::string> parseArgs(int argc, char* argv[]) {
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc - 1; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) == 0) { args[key] = argv[i+1]; ++i; }
    }
    return args;
}

int main(int argc, char* argv[]) {
    std::cout << "=== Drone FHE 3D - Batching ===" << std::endl;
    std::cout.flush();

    // ── Args ──────────────────────────────────────────────────────────────────
    auto args  = parseArgs(argc, argv);
    std::string f1 = args.count("--path1") ? args["--path1"] : "";
    std::string f2 = args.count("--path2") ? args["--path2"] : "";
    std::string f3 = args.count("--path3") ? args["--path3"] : "";

    if (f1.empty() || f2.empty()) {
        std::cerr << "Usage: ./drone_fhe --path1 f1.txt --path2 f2.txt [--path3 f3.txt]\n";
        return 1;
    }

    // ── Charger trajectoires ──────────────────────────────────────────────────
    std::vector<Path>        paths;
    std::vector<std::string> names;
    try {
        paths.push_back(PathIO::read_single_path(f1)); names.push_back("Drone 1");
        paths.push_back(PathIO::read_single_path(f2)); names.push_back("Drone 2");
        if (!f3.empty()) {
            paths.push_back(PathIO::read_single_path(f3)); names.push_back("Drone 3");
        }
        for (size_t i = 0; i < paths.size(); ++i)
            std::cout << "  " << names[i] << " : " << paths[i].size() << " points\n";
    } catch (const std::exception& e) {
        std::cerr << "Erreur lecture : " << e.what() << "\n";
        return 1;
    }
    std::cout.flush();

    // ── Init FHE ──────────────────────────────────────────────────────────────
    std::cout << "\n--- Init FHE ---\n"; std::cout.flush();
    auto t0 = std::chrono::high_resolution_clock::now();

    CryptoEngine engine;
    engine.initialize();   // paramètres par défaut du Config()
    engine.setupSchemeSwitching();
    GeometryEngine geo(&engine);

    auto t_ready = std::chrono::high_resolution_clock::now();
    double ms_setup = std::chrono::duration<double,std::milli>(t_ready - t0).count();
    std::cout << "  Setup : " << std::fixed << std::setprecision(1)
              << ms_setup/1000 << " s\n";
    std::cout.flush();

    // ── Detection par paires avec batching ───────────────────────────────────
    std::cout << "\n--- Detection FHE (batching) ---\n"; std::cout.flush();

    // Structure résultat par paire
    struct PairResult {
        std::string name;
        bool        collision;
        double      timeMs;
    };
    std::vector<PairResult> pairResults;
    int totalCollisions = 0;

    // Pour chaque drone i, on teste ses segments contre tous les segments
    // des autres drones en batch
    for (size_t i = 0; i < paths.size(); ++i) {
        for (size_t j = i + 1; j < paths.size(); ++j) {
            std::string pairName = names[i] + " x " + names[j];
            std::cout << "\n  -- " << pairName << " --\n"; std::cout.flush();

            // Référence en clair
            IntersectionDetector detector(&engine, &geo);
            auto clearRes = detector.detectIntersectionsClear(paths[i], paths[j]);
            int clearHits = 0;
            for (auto& r : clearRes) if (r.intersects) ++clearHits;
            std::cout << "    Clair  : " << clearHits << " collision(s)\n";
            std::cout.flush();

            // Batch FHE : 1er segment de paths[i] comme référence,
            // tous les segments de paths[j] comme voisins
            Segment mySeg = {paths[i][0], paths[i][1]};
            std::vector<Segment> neighbors;
            for (size_t k = 0; k+1 < paths[j].size(); ++k)
                neighbors.push_back({paths[j][k], paths[j][k+1]});

            auto tb0 = std::chrono::high_resolution_clock::now();
            auto batchRes = geo.batchCheckIntersection3D(mySeg, neighbors);
            auto tb1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double,std::milli>(tb1-tb0).count();

            bool fheCollision = false;
            for (auto v : batchRes) if (v > 0.5) { fheCollision = true; break; }

            std::cout << "    FHE    : " << (fheCollision ? "COLLISION" : "LIBRE")
                      << "  (" << std::fixed << std::setprecision(0) << ms << " ms)\n";
            std::cout.flush();

            pairResults.push_back({pairName, (clearHits > 0), ms});
            if (clearHits > 0) ++totalCollisions;
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double ms_total = std::chrono::duration<double,std::milli>(t_end-t0).count();

    // ── Stats ─────────────────────────────────────────────────────────────────
    std::cout << "\n--- Resume ---\n";
    std::cout << "  Drones     : " << paths.size() << "\n";
    std::cout << "  Paires     : " << pairResults.size() << "\n";
    std::cout << "  Collisions : " << totalCollisions << "\n";
    std::cout << "  Temps total: " << std::fixed << std::setprecision(1)
              << ms_total/1000 << " s\n";
    geo.printStats();
    std::cout.flush();

    // ── JSON pour server.py ───────────────────────────────────────────────────
    std::cout << "\nJSON_RESULT:{"
              << "\"collision\":"       << (totalCollisions > 0 ? "true" : "false") << ","
              << "\"collisions_count\":" << totalCollisions << ","
              << "\"pairs_tested\":"    << pairResults.size() << ","
              << "\"drones\":"          << paths.size() << ","
              << "\"scheme_switches\":" << geo.getBootstrapCount() << ","
              << "\"total_time_ms\":"   << std::fixed << std::setprecision(1) << ms_total << ","
              << "\"details\":[";

    for (size_t k = 0; k < pairResults.size(); ++k) {
        if (k > 0) std::cout << ",";
        std::cout << "{"
                  << "\"pair\":\"" << pairResults[k].name << "\","
                  << "\"collision\":" << (pairResults[k].collision ? "true" : "false") << ","
                  << "\"time_ms\":" << std::fixed << std::setprecision(1) << pairResults[k].timeMs
                  << "}";
    }
    std::cout << "]}\n";
    std::cout.flush();

    return 0;
}
