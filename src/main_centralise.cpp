#include "engine.hpp"
#include "geometry.hpp"
#include "intersection.hpp"
#include "io.hpp"
#include <chrono>
#include <iostream>
#include <exception>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{
    std::string path1_file, path2_file;
    std::string data_dir = "data";
    int scenario = 0;
    bool json_mode = false;

    for (int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "--path1" && i + 1 < argc)
            path1_file = argv[++i];
        else if (arg == "--path2" && i + 1 < argc)
            path2_file = argv[++i];
        else if (arg == "--data" && i + 1 < argc)
            data_dir = argv[++i];
        else if (arg == "--scenario" && i + 1 < argc)
            scenario = std::stoi(argv[++i]);
        else if (arg == "--json")
            json_mode = true;
        else if (arg == "--no-bench") { /* ignoré */ }
        else if (arg == "--help")
        {
            std::cout << "Usage: drone_collision_centralise [OPTIONS]\n\n"
                      << "Options:\n"
                      << "  --path1 <f>   Trajectoire drone 1\n"
                      << "  --path2 <f>   Trajectoire drone 2\n"
                      << "  --data  <d>   Dossier data/ (defaut: ./data)\n"
                      << "  --scenario <1|2|4|5|6>\n"
                      << "      1 = 2 drones en collision       (route1 x route2, z=10)\n"
                      << "      2 = 2 drones sans collision     (route1 x route3, altitudes diff)\n"
                      << "      4 = 6 drones altitudes mixtes   (3 collisions / 15 paires)\n"
                      << "      5 = 6 drones grille z=10        (9 collisions / 15 paires)\n"
                      << "      6 = 6 drones etoile z=15        (15 collisions / 15 paires)\n"
                      << "  --json        Sortie JSON\n";
            return 0;
        }
    }

    // scenarios 4/5/6 = multi-drone
    bool multi_drone = (scenario >= 4);

    // prefixes des fichiers pour chaque scenario multi-drone
    // scenario 4 : cryptroute1..6  (mix altitudes, 3 collisions)
    // scenario 5 : grid_drone1..6  (grille orthogonale z=10, 9 collisions)
    // scenario 6 : star_drone1..6  (etoile convergente z=15, 15 collisions)
    std::string multi_prefix;
    if      (scenario == 4) multi_prefix = data_dir + "/cryptroute";
    else if (scenario == 5) multi_prefix = data_dir + "/grid_drone";
    else if (scenario == 6) multi_prefix = data_dir + "/star_drone";

    if (scenario == 1) { path1_file = data_dir + "/cryptroute1.txt";
                         path2_file = data_dir + "/cryptroute2.txt"; }
    else if (scenario == 2) { path1_file = data_dir + "/cryptroute1.txt";
                              path2_file = data_dir + "/cryptroute3.txt"; }

    try
    {
        if (!json_mode)
            std::cout << "=== Drone Collision Detection (Encrypted) ===\n";

        // ── Initialisation crypto ──────────────────────────────────────────────
        CryptoEngine engine;
        CryptoEngine::Config config;
        config.batchSize    = 16;
        config.switchValues = 4;
        config.logQ_ccLWE   = 21;

        if (!json_mode) std::cout << "[*] Initializing CryptoEngine...\n";
        engine.initialize(config);
        if (!json_mode) std::cout << "[*] Setting up Scheme Switching...\n";
        engine.setupSchemeSwitching();

        GeometryEngine geometry(&engine);
        IntersectionDetector detector(&engine, &geometry);

        // ══════════════════════════════════════════════════════════════════════
        // ══════════════════════════════════════════════════════════════════════
        // SCENARIOS 4/5/6 — DRONE INITIATEUR vs N VOISINS
        //
        // Drone 1 (l'initiateur) verifie sa propre trajectoire contre les N-1
        // autres drones en UN SEUL appel FHE batche.
        // On ne teste PAS les collisions entre les autres drones (drone i vs drone j).
        // ══════════════════════════════════════════════════════════════════════
        if (multi_drone)
        {
            const int N_DRONES = 6;
            std::vector<Path> paths(N_DRONES);

            const char* scen_labels[] = {"","","","",
                "altitudes mixtes  — attendu : 2 collisions",
                "grille z=10       — attendu : 3 collisions (drone1 vs vertical)",
                "etoile z=15       — attendu : 5 collisions (drone1 vs tous)"};
            if (!json_mode)
                std::cout << "[*] Scenario " << scenario << " : Drone 1 initiateur, "
                          << N_DRONES-1 << " voisins\n"
                          << "    " << scen_labels[scenario] << "\n";

            for (int d = 0; d < N_DRONES; d++)
            {
                std::string f = multi_prefix + std::to_string(d+1) + ".txt";
                paths[d] = PathIO::read_single_path(f);
                if (!json_mode)
                    PathIO::print_path(paths[d],
                        d == 0 ? "Drone 1 [INITIATEUR]"
                               : "Drone " + std::to_string(d+1) + " [voisin]");
            }

            // ── construire la liste UNIQUE de tous les segments voisins ────────
            // tous les segments des drones 2..N dans un seul vecteur
            // batchCheckIntersection3D les testera tous en parallele (1 seul appel)
            std::vector<Segment> all_neighbors;
            std::vector<int>     seg_to_drone;  // drone source de chaque segment voisin
            for (int d = 1; d < N_DRONES; d++)
                for (size_t s = 0; s + 1 < paths[d].size(); s++)
                {
                    all_neighbors.push_back({paths[d][s], paths[d][s+1]});
                    seg_to_drone.push_back(d + 1);  // drone 2..N (1-indexed)
                }
            int N_neighbors = (int)all_neighbors.size();

            // ── test en clair (reference) ──────────────────────────────────────
            if (!json_mode) std::cout << "\n[*] Test en clair (reference, drone 1 vs voisins)...\n";
            int clear_col = 0;
            for (int d = 1; d < N_DRONES; d++)
            {
                auto r = detector.detectIntersectionsClear(paths[0], paths[d]);
                if (!r.empty()) { clear_col++;
                    if (!json_mode) std::cout << "  [CLAIR] Drone 1 x Drone " << d+1 << " : COLLISION\n"; }
                else
                    if (!json_mode) std::cout << "  [CLAIR] Drone 1 x Drone " << d+1 << " : libre\n";
            }
            if (!json_mode) std::cout << "  -> " << clear_col << " collision(s) detectee(s) en clair\n";

            // ── test FHE : 1 seul appel batch par segment de drone 1 ───────────
            // tous les N_neighbors segments voisins sont testes en parallele
            // grace au batching SIMD : N_neighbors slots dans 1 seul ciphertext
            if (!json_mode)
                std::cout << "\n[*] Test FHE (drone 1 vs " << N_neighbors
                          << " segments voisins en 1 batch)...\n";

            auto t0 = std::chrono::high_resolution_clock::now();
            int fhe_col = 0;
            size_t total_pairs = 0;

            for (size_t s = 0; s + 1 < paths[0].size(); s++)
            {
                Segment mySeg = {paths[0][s], paths[0][s+1]};

                // UN SEUL appel : tous les voisins testes en parallele
                auto res = geometry.batchCheckIntersection3D(mySeg, all_neighbors);
                total_pairs += res.size();

                for (int t = 0; t < (int)res.size(); t++)
                {
                    if (res[t] > 0.5)
                    {
                        fhe_col++;
                        if (!json_mode)
                            std::cout << "  [FHE] Drone 1 seg[" << s
                                      << "] x Drone " << seg_to_drone[t]
                                      << " : COLLISION\n";
                    }
                }
            }

            auto t1 = std::chrono::high_resolution_clock::now();
            double fhe_ms = std::chrono::duration<double,std::milli>(t1 - t0).count();

            size_t actual_ss       = geometry.getBootstrapCount();
            // sans batching : N_neighbors × 9 SS par segment de drone1
            size_t theo_ss_nobatch = total_pairs * 9;
            size_t gain_ss         = theo_ss_nobatch / std::max((size_t)1, actual_ss);
            double ms_per_ss       = fhe_ms / std::max((size_t)1, actual_ss);

            if (json_mode)
            {
                std::cout << "{\n"
                          << "  \"scenario\": " << scenario << ",\n"
                          << "  \"n_drones\": " << N_DRONES << ",\n"
                          << "  \"n_neighbors\": " << N_neighbors << ",\n"
                          << "  \"collision_fhe\": " << fhe_col << ",\n"
                          << "  \"collision_clear\": " << clear_col << ",\n"
                          << "  \"segments_tested\": " << total_pairs << ",\n"
                          << "  \"time_fhe_ms\": " << fhe_ms << ",\n"
                          << "  \"ss_batch\": " << actual_ss << ",\n"
                          << "  \"ss_nobatch\": " << theo_ss_nobatch << ",\n"
                          << "  \"gain_ss\": " << gain_ss << "\n"
                          << "}\n";
            }
            else
            {
                std::cout << "\n=== RESULTAT : DRONE 1 vs " << N_neighbors << " VOISINS ===\n";
                std::cout << "  Collisions (FHE)  : " << fhe_col   << "\n";
                std::cout << "  Collisions (clair): " << clear_col << "\n";
                std::cout << "\n+----------------------------------------------------+\n";
                std::cout << "|          AVANTAGE DU BATCHING FHE                  |\n";
                std::cout << "+----------------------------------------------------+\n";
                std::cout << "| Drones voisins             : " << N_DRONES-1        << "\n";
                std::cout << "| Segments voisins (batch)   : " << N_neighbors        << "\n";
                std::cout << "| Appels FHE                 : "
                          << paths[0].size()-1 << " (1 par segment drone 1)\n";
                std::cout << "+----------------------------------------------------+\n";
                std::cout << "| SCHEME-SWITCHES                                    |\n";
                std::cout << "|   Sans batching (theorique): " << theo_ss_nobatch
                          << " SS (" << N_neighbors << " voisins x "
                          << paths[0].size()-1 << " segs x 9)\n";
                std::cout << "|   Avec batching (reel)     : " << actual_ss << " SS\n";
                std::cout << "|   GAIN                     : x" << gain_ss << "\n";
                std::cout << "+----------------------------------------------------+\n";
                std::cout << "| TEMPS FHE TOTAL            : " << fhe_ms << " ms\n";
                std::cout << "| Temps moyen / SS           : " << ms_per_ss << " ms\n";
                std::cout << "+----------------------------------------------------+\n";
                geometry.printStats();
            }
            return 0;
        }

        // ══════════════════════════════════════════════════════════════════════
        // SCENARIOS 1/2 : 2 drones (code original)
        // ══════════════════════════════════════════════════════════════════════
        Path path1, path2;

        if (!path1_file.empty() && !path2_file.empty())
        {
            if (!json_mode)
            {
                const char *labels[] = {"", "COLLISION (meme altitude)",
                                        "PAS DE COLLISION (altitudes differentes)", ""};
                if (scenario > 0 && scenario < 3)
                    std::cout << "[*] Scenario " << scenario << " : " << labels[scenario] << "\n";
                std::cout << "[*] Lecture : " << path1_file << "\n";
                std::cout << "[*] Lecture : " << path2_file << "\n";
            }
            path1 = PathIO::read_single_path(path1_file);
            path2 = PathIO::read_single_path(path2_file);
        }
        else
        {
            if (!json_mode) std::cout << "[*] Chemins aleatoires generes\n";
            path1 = PathIO::generate_random_path(5, 42);
            path2 = PathIO::generate_random_path(5, 1337);
        }

        if (!json_mode)
        {
            PathIO::print_path(path1, "Drone 1");
            PathIO::print_path(path2, "Drone 2");
        }

        // test en clair
        if (!json_mode) std::cout << "\n[*] Testing CLEAR intersections...\n";
        auto start = std::chrono::high_resolution_clock::now();
        auto clearResults = detector.detectIntersectionsClear(path1, path2);
        auto end = std::chrono::high_resolution_clock::now();
        long clear_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (!json_mode)
        {
            std::cout << "Clear intersections found: " << clearResults.size() << "\n";
            std::cout << "Clear processing time: " << clear_ms << " ms\n";
        }

        // test FHE
        if (!json_mode) std::cout << "\n[*] Testing ENCRYPTED intersections (BATCH 3D)...\n";
        auto start2 = std::chrono::high_resolution_clock::now();

        int totalCollisions = 0;
        std::vector<std::pair<size_t,size_t>> collisionPairs;

        std::vector<Segment> neighbors;
        for (size_t j = 0; j < path2.size()-1; j++)
            neighbors.push_back({path2[j], path2[j+1]});

        for (size_t i = 0; i < path1.size()-1; i++)
        {
            Segment mySeg = {path1[i], path1[i+1]};
            auto results = geometry.batchCheckIntersection3D(mySeg, neighbors);
            for (size_t j = 0; j < results.size(); j++)
            {
                if (results[j] > 0.5)
                {
                    totalCollisions++;
                    collisionPairs.push_back({i,j});
                    if (!json_mode)
                        std::cout << "  Collision: seg[" << i << "] x seg[" << j << "]\n";
                }
            }
        }

        auto end2 = std::chrono::high_resolution_clock::now();
        double duration_sec = std::chrono::duration<double>(end2-start2).count();
        size_t segments_tested = (path1.size()-1)*(path2.size()-1);

        size_t actual_ss       = geometry.getBootstrapCount();
        size_t theo_ss_nobatch = segments_tested * 9;
        size_t gain_ss         = theo_ss_nobatch / std::max((size_t)1, actual_ss);
        double ss_ms_reel      = (duration_sec*1000.0) / std::max((size_t)1, actual_ss);

        if (json_mode)
        {
            std::cout << "{\n"
                      << "  \"collision\": " << (totalCollisions>0?"true":"false") << ",\n"
                      << "  \"collision_count\": " << totalCollisions << ",\n"
                      << "  \"segments_tested\": " << segments_tested << ",\n"
                      << "  \"time_ms\": " << (duration_sec*1000.0) << ",\n"
                      << "  \"time_clear_ms\": " << clear_ms << ",\n"
                      << "  \"scheme_switches_batch\": " << actual_ss << ",\n"
                      << "  \"scheme_switches_nobatch\": " << theo_ss_nobatch << ",\n"
                      << "  \"batching_gain_ss\": " << gain_ss << ",\n"
                      << "  \"details\": [";
            for (size_t k = 0; k < collisionPairs.size(); k++)
            {
                if (k>0) std::cout << ", ";
                std::cout << "{\"seg1\":" << collisionPairs[k].first
                          << ",\"seg2\":" << collisionPairs[k].second << "}";
            }
            std::cout << "]\n}\n";
        }
        else
        {
            std::cout << "\n=== RESULTAT FINAL ===\n";
            if (totalCollisions > 0)
                std::cout << "  COLLISION DETECTEE (" << totalCollisions << " paire(s))\n";
            else
                std::cout << "  AUCUNE COLLISION\n";

            std::cout << "\n+--------------------------------------------------+\n";
            std::cout << "|         AVANTAGES DU BATCHING FHE                |\n";
            std::cout << "+--------------------------------------------------+\n";
            std::cout << "| Paires de segments testees  : " << segments_tested << "\n";
            std::cout << "+--------------------------------------------------+\n";
            std::cout << "| SCHEME-SWITCHES                                  |\n";
            std::cout << "|   Sans batching (theorique) : " << theo_ss_nobatch << " SS\n";
            std::cout << "|   Avec batching (reel)      : " << actual_ss << " SS\n";
            std::cout << "|   GAIN                      : x" << gain_ss << "\n";
            std::cout << "+--------------------------------------------------+\n";
            std::cout << "| TEMPS REEL MESURE                                |\n";
            std::cout << "|   En clair                  : " << clear_ms << " ms\n";
            std::cout << "|   FHE batch (mesure)        : " << (duration_sec*1000.0) << " ms\n";
            std::cout << "|   Temps moyen / SS          : " << ss_ms_reel << " ms\n";
            std::cout << "|   Overhead FHE vs clair     : x"
                      << (duration_sec*1000.0/std::max(1L,clear_ms)) << "\n";
            std::cout << "+--------------------------------------------------+\n";
            detector.printStatistics();
            geometry.printStats();
        }

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
