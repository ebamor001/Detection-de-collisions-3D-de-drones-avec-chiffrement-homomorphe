#include "engine.hpp"
#include "geometry.hpp"
#include "io.hpp"

#include <chrono>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    std::string path1_file = "data/cryptroute1.txt";
    std::string path2_file = "data/cryptroute2.txt";

    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg == "--path1" && i + 1 < argc) path1_file = argv[++i];
        else if (arg == "--path2" && i + 1 < argc) path2_file = argv[++i];
    }

    try {
        std::cout << "=== Full FHE single-segment collision test ===\n";

        CryptoEngine engine;
        CryptoEngine::Config config;
        config.batchSize = 64;
        config.switchValues = 64;
        config.logQ_ccLWE = 25;

        engine.initialize(config);
        engine.setupSchemeSwitching();

        GeometryEngine geometry(&engine);

        Path path1 = PathIO::read_single_path(path1_file);
        Path path2 = PathIO::read_single_path(path2_file);

        if (path1.size() < 2 || path2.size() < 2) {
            std::cerr << "Erreur : chaque trajectoire doit avoir au moins 2 points.\n";
            return 1;
        }

        Segment s1 = {path1[0], path1[1]};
        Segment s2 = {path2[0], path2[1]};

        std::cout << "Segment Drone 1 : "
                  << "(" << s1.first.x << "," << s1.first.y << "," << s1.first.z << ") -> "
                  << "(" << s1.second.x << "," << s1.second.y << "," << s1.second.z << ")\n";

        std::cout << "Segment Drone 2 : "
                  << "(" << s2.first.x << "," << s2.first.y << "," << s2.first.z << ") -> "
                  << "(" << s2.second.x << "," << s2.second.y << "," << s2.second.z << ")\n";

        auto ct_p1x = engine.encryptValue(s1.first.x);
        auto ct_p1y = engine.encryptValue(s1.first.y);
        auto ct_p1z = engine.encryptValue(s1.first.z);
        auto ct_q1x = engine.encryptValue(s1.second.x);
        auto ct_q1y = engine.encryptValue(s1.second.y);
        auto ct_q1z = engine.encryptValue(s1.second.z);

        auto ct_p2x = engine.encryptValue(s2.first.x);
        auto ct_p2y = engine.encryptValue(s2.first.y);
        auto ct_p2z = engine.encryptValue(s2.first.z);
        auto ct_q2x = engine.encryptValue(s2.second.x);
        auto ct_q2y = engine.encryptValue(s2.second.y);
        auto ct_q2z = engine.encryptValue(s2.second.z);

        auto t0 = std::chrono::high_resolution_clock::now();

        auto ct_result = geometry.checkSegmentIntersection3DEncrypted(
            ct_p1x, ct_p1y, ct_p1z,
            ct_q1x, ct_q1y, ct_q1z,
            ct_p2x, ct_p2y, ct_p2z,
            ct_q2x, ct_q2y, ct_q2z
        );

        double result = engine.decryptValue(ct_result);
        bool collision = result > 0.5;

        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::cout << "\n=== RESULTAT FULL FHE ===\n";
        std::cout << (collision ? "COLLISION" : "LIBRE") << "\n";
        std::cout << "Temps : " << ms << " ms\n";

        geometry.printStats();

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
}