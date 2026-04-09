#include "engine.hpp"
#include "geometry.hpp"
#include "intersection.hpp"
#include "io.hpp"
#include <chrono>
#include <iostream>
#include <exception>
#include <string>

int main(int argc, char* argv[])
{
    // =========================
    // 0. PARSE ARGUMENTS CLI
    // =========================
    std::string path1_file, path2_file;
    std::string data_dir = "data"; // dossier data/ par défaut
    int scenario = 0;              // 0 = pas de scénario prédéfini
    bool json_mode = false;

    for (int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if      (arg == "--path1"    && i + 1 < argc) path1_file = argv[++i];
        else if (arg == "--path2"    && i + 1 < argc) path2_file = argv[++i];
        else if (arg == "--data"     && i + 1 < argc) data_dir   = argv[++i];
        else if (arg == "--scenario" && i + 1 < argc) scenario   = std::stoi(argv[++i]);
        else if (arg == "--json")    json_mode = true;
        else if (arg == "--no-bench") { /* ignoré */ }
        else if (arg == "--help")
        {
            std::cout << "Usage: drone_collision [OPTIONS]\n\n"
                      << "Options:\n"
                      << "  --path1 <fichier>    Trajectoire drone 1 (format: (x,y,z)(x,y,z)...)\n"
                      << "  --path2 <fichier>    Trajectoire drone 2\n"
                      << "  --data  <dossier>    Dossier data/ (defaut: ./data)\n"
                      << "  --scenario <1|2|3>   Scenario predéfini :\n"
                      << "      1 = Collision     (route1 x route2, meme altitude z=10)\n"
                      << "      2 = Pas collision  (route1 x route3, altitudes differentes)\n"
                      << "      3 = Zigzag        (route4 x route5, trajectoires complexes)\n"
                      << "  --json               Sortie JSON (pour server.py)\n"
                      << "  (sans option)        Chemins aleatoires generes automatiquement\n";
            return 0;
        }
    }

    // Scénarios prédéfinis depuis data/
    if (scenario == 1) { path1_file = data_dir + "/cryptroute1.txt";
                         path2_file = data_dir + "/cryptroute2.txt"; }
    else if (scenario == 2) { path1_file = data_dir + "/cryptroute1.txt";
                              path2_file = data_dir + "/cryptroute3.txt"; }
    else if (scenario == 3) { path1_file = data_dir + "/cryptroute4.txt";
                              path2_file = data_dir + "/cryptroute5.txt"; }

    try
    {
        if (!json_mode)
            std::cout << "=== Drone Collision Detection (Encrypted) ===\n";

        // =========================
        // 1. INITIALISATION CRYPTO
        // =========================
        CryptoEngine engine;
        CryptoEngine::Config config;
        // Valeurs réduites pour la démo — switchValues=4096 rendrait setupSchemeSwitching très lent (>5 min)
        config.batchSize    = 64;
        config.switchValues = 64;
        config.logQ_ccLWE   = 25;

        if (!json_mode) std::cout << "[*] Initializing CryptoEngine...\n";
        engine.initialize(config);

        if (!json_mode) std::cout << "[*] Setting up Scheme Switching...\n";
        engine.setupSchemeSwitching();

        // =========================
        // 2. GEOMETRY + DETECTOR
        // =========================
        GeometryEngine geometry(&engine);
        IntersectionDetector detector(&engine, &geometry);

        // =========================
        // 3. LIRE LES PATHS (CLI) OU GÉNÉRER (mode test)
        // =========================
        Path path1, path2;

        if (!path1_file.empty() && !path2_file.empty())
        {
            if (!json_mode)
            {
                if (scenario > 0)
                {
                    const char* labels[] = {"", "COLLISION (meme altitude)", "PAS DE COLLISION (altitudes differentes)", "ZIGZAG complexe"};
                    std::cout << "[*] Scenario " << scenario << " : " << labels[scenario] << "\n";
                }
                std::cout << "[*] Lecture : " << path1_file << "\n";
                std::cout << "[*] Lecture : " << path2_file << "\n";
            }
            path1 = PathIO::read_single_path(path1_file);
            path2 = PathIO::read_single_path(path2_file);
        }
        else
        {
            if (!json_mode) std::cout << "[*] Aucun fichier specifie — chemins aleatoires generes\n";
            path1 = PathIO::generate_random_path(5, 42);
            path2 = PathIO::generate_random_path(5, 1337);
        }

        if (!json_mode)
        {
            PathIO::print_path(path1, "Drone 1");
            PathIO::print_path(path2, "Drone 2");
        }

        // =========================
        // 4. TEST EN CLAIR (REFERENCE)
        // =========================
        if (!json_mode) std::cout << "\n[*] Testing CLEAR intersections...\n";
        auto start = std::chrono::high_resolution_clock::now();
        auto clearResults = detector.detectIntersectionsClear(path1, path2);
        auto end = std::chrono::high_resolution_clock::now();
        auto clear_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (!json_mode)
        {
            std::cout << "Clear intersections found: " << clearResults.size() << "\n";
            std::cout << "Clear processing time: " << clear_ms << " ms\n";
        }

        // =========================
        // 5. TEST CHIFFRE (HYBRID CKKS+TFHE)
        // =========================
        if (!json_mode) std::cout << "\n[*] Testing ENCRYPTED intersections (BATCH 3D)...\n";

        auto start2 = std::chrono::high_resolution_clock::now();

        int totalCollisions = 0;
        std::vector<std::pair<size_t,size_t>> collisionPairs;

        std::vector<Segment> neighbors;
        for (size_t j = 0; j < path2.size() - 1; j++)
            neighbors.push_back({path2[j], path2[j + 1]});

        // boucle principale
        for (size_t i = 0; i < path1.size() - 1; i++)
        {
            Segment mySeg = {path1[i], path1[i + 1]};
            auto results = geometry.batchCheckIntersection3D(mySeg, neighbors);

            for (size_t j = 0; j < results.size(); j++)
            {
                if (results[j] > 0.5)
                {
                    totalCollisions++;
                    collisionPairs.push_back({i, j});
                    if (!json_mode)
                        std::cout << "  Collision: seg[" << i << "] x seg[" << j << "]\n";
                }
            }
        }

        auto end2 = std::chrono::high_resolution_clock::now();
        double duration_sec = std::chrono::duration<double>(end2 - start2).count();
        size_t segments_tested = (path1.size() - 1) * (path2.size() - 1);

        // =========================
        // 6. CALCUL DES STATS BATCHING
        // =========================
        size_t actual_ss       = geometry.getBootstrapCount();
        size_t theo_ss_nobatch = segments_tested * 9; // 9 SS/paire sans batching
        size_t gain_ss         = theo_ss_nobatch / std::max((size_t)1, actual_ss);

        // temps estimé sans batching (820 ms par scheme-switch + 45s keygen)
        double keygen_ms       = 45000.0;
        double ss_ms           = 820.0;
        double time_nobatch_ms = keygen_ms + theo_ss_nobatch * ss_ms;
        double time_batch_ms   = keygen_ms + actual_ss * ss_ms;
        double gain_time       = time_nobatch_ms / std::max(1.0, time_batch_ms);

        // =========================
        // 7. SORTIE JSON ou TEXTE
        // =========================
        if (json_mode)
        {
            std::cout << "{\n";
            std::cout << "  \"collision\": " << (totalCollisions > 0 ? "true" : "false") << ",\n";
            std::cout << "  \"collision_count\": " << totalCollisions << ",\n";
            std::cout << "  \"segments_tested\": " << segments_tested << ",\n";
            std::cout << "  \"time_ms\": " << (duration_sec * 1000.0) << ",\n";
            std::cout << "  \"time_clear_ms\": " << clear_ms << ",\n";
            std::cout << "  \"scheme_switches_batch\": " << actual_ss << ",\n";
            std::cout << "  \"scheme_switches_nobatch\": " << theo_ss_nobatch << ",\n";
            std::cout << "  \"batching_gain_ss\": " << gain_ss << ",\n";
            std::cout << "  \"details\": [";
            for (size_t k = 0; k < collisionPairs.size(); k++)
            {
                if (k > 0) std::cout << ", ";
                std::cout << "{\"seg1\": " << collisionPairs[k].first
                          << ", \"seg2\": " << collisionPairs[k].second << "}";
            }
            std::cout << "]\n}\n";
        }
        else
        {
            std::cout << "\n";
            std::cout << "=== RESULTAT FINAL ===\n";
            if (totalCollisions > 0)
                std::cout << "  COLLISION DETECTEE (" << totalCollisions << " paire(s))\n";
            else
                std::cout << "  AUCUNE COLLISION\n";

            // =========================
            // 8. STATS BATCHING DETAILLEES
            // =========================
            std::cout << "\n";
            std::cout << "+--------------------------------------------------+\n";
            std::cout << "|         AVANTAGES DU BATCHING FHE                |\n";
            std::cout << "+--------------------------------------------------+\n";
            std::cout << "| Paires de segments testees  : " << segments_tested << "\n";
            std::cout << "+--------------------------------------------------+\n";
            std::cout << "| SCHEME-SWITCHES                                  |\n";
            std::cout << "|   Sans batching (theorique) : " << theo_ss_nobatch
                      << " SS  (" << segments_tested << " x 9)\n";
            std::cout << "|   Avec batching (reel)      : " << actual_ss
                      << " SS  (2 copla + 4 orient)\n";
            std::cout << "|   GAIN                      : x" << gain_ss << "\n";
            std::cout << "+--------------------------------------------------+\n";
            std::cout << "| TEMPS ESTIME (820 ms/SS + 45s keygen)            |\n";
            std::cout << "|   Sans batching             : "
                      << (time_nobatch_ms / 1000.0) << " s\n";
            std::cout << "|   Avec batching             : "
                      << (time_batch_ms / 1000.0) << " s\n";
            std::cout << "|   GAIN                      : x" << gain_time << "\n";
            std::cout << "+--------------------------------------------------+\n";
            std::cout << "| TEMPS REEL MESURE                                |\n";
            std::cout << "|   En clair                  : " << clear_ms << " ms\n";
            std::cout << "|   FHE batch (mesure)        : "
                      << (duration_sec * 1000.0) << " ms\n";
            std::cout << "|   Overhead FHE vs clair     : x"
                      << (duration_sec * 1000.0 / std::max(1L, clear_ms)) << "\n";
            std::cout << "+--------------------------------------------------+\n";

            // ASCII bar chart : SS avec vs sans batching
            std::cout << "\n  Scheme-switches (barre = 1 SS) :\n";
            std::cout << "  Avec    [";
            for (size_t s = 0; s < actual_ss && s < 20; s++) std::cout << "#";
            std::cout << "] " << actual_ss << " SS\n";
            std::cout << "  Sans    [";
            size_t bar = std::min(theo_ss_nobatch, (size_t)20);
            for (size_t s = 0; s < bar; s++) std::cout << "#";
            if (theo_ss_nobatch > 20) std::cout << "...";
            std::cout << "] " << theo_ss_nobatch << " SS\n";

            // Données CSV pour le script de courbes
            std::cout << "\n  [CSV] N,ss_batch,ss_nobatch,gain\n";
            for (size_t n = 1; n <= 20; n++)
            {
                size_t ss_b  = 6;           // constant avec batching
                size_t ss_nb = n * 9;
                std::cout << "  " << n << "," << ss_b << "," << ss_nb
                          << "," << (ss_nb / ss_b) << "\n";
            }

            // =========================
            // 9. STATS DETECTOR + GEOMETRY
            // =========================
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