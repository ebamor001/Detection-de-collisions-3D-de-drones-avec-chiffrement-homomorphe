#include "engine.hpp"
#include "geometry.hpp"
#include <chrono>
#include <iostream>

int main() {
    try {
        // =============================
        // 1. Init CryptoEngine
        // =============================
        CryptoEngine engine;
        CryptoEngine::Config cfg;

        engine.initialize(cfg);
        engine.setupSchemeSwitching();

        GeometryEngine geom(&engine);
        
        cfg.multDepth = 20;
        cfg.scaleModSize = 50;
        cfg.batchSize = 4096;
        cfg.ringDim = 8192;
        cfg.secLevel = HEStd_NotSet;

        // params TFHE (pour compare)
        cfg.slBin = STD128;
        cfg.logQ_ccLWE = 25;
        cfg.scaleSign = 1.0;
        cfg.guardGain = 1.0;



        // =============================
        // 2. Définir segments test
        // =============================
        IntPoint p1{0,0,0};
        IntPoint q1{10,10,0};

        IntPoint p2{0,10,0};
        IntPoint q2{10,0,0};

        // version clear (debug)
        bool clearRes = geom.doSegmentsIntersectClear3D({p1,q1}, {p2,q2});
        std::cout << "Clear result: " << clearRes << std::endl;

        // =============================
        // 3. Encrypt points
        // =============================
        auto p1x = engine.encryptValue(p1.x);
        auto p1y = engine.encryptValue(p1.y);
        auto p1z = engine.encryptValue(p1.z);

        auto q1x = engine.encryptValue(q1.x);
        auto q1y = engine.encryptValue(q1.y);
        auto q1z = engine.encryptValue(q1.z);

        auto p2x = engine.encryptValue(p2.x);
        auto p2y = engine.encryptValue(p2.y);
        auto p2z = engine.encryptValue(p2.z);

        auto q2x = engine.encryptValue(q2.x);
        auto q2y = engine.encryptValue(q2.y);
        auto q2z = engine.encryptValue(q2.z);

        // =============================
        // 4. Benchmark
        // =============================
        int runs = 3;

        for (int i = 0; i < runs; i++) {

            auto start = std::chrono::high_resolution_clock::now();

            auto res = geom.checkSegmentIntersection3DEncrypted(
                p1x,p1y,p1z,
                q1x,q1y,q1z,
                p2x,p2y,p2z,
                q2x,q2y,q2z
            );

            auto end = std::chrono::high_resolution_clock::now();

            double time = std::chrono::duration<double>(end - start).count();

            double val = engine.decryptValue(res);

            std::cout << "\nRun " << i << std::endl;
            std::cout << "Time: " << time << " s" << std::endl;
            std::cout << "Intersection (HE): " << val << std::endl;
        }

        // =============================
        // 5. Stats
        // =============================
        geom.printStats();

        std::cout << "Avg SS per test: "
                  << (double)geom.getBootstrapCount() / geom.getIntersectionTests()
                  << std::endl;

        // =============================
        // 6. TEST BATCHING
        // =============================
        std::vector<Segment> neighbors;

        for (int i = 0; i < 20; i++) {
            IntPoint a{0+i, 10, 10};
            IntPoint b{10+i, 0, 0};
            neighbors.push_back({a,b});
        }

        Segment mySeg{p1,q1};

        auto startBatch = std::chrono::high_resolution_clock::now();

        auto results = geom.batchCheckIntersection3D(mySeg, neighbors);

        auto endBatch = std::chrono::high_resolution_clock::now();

        std::cout << "\nBatch time: "
                  << std::chrono::duration<double>(endBatch - startBatch).count()
                  << " s" << std::endl;

        for (size_t i = 0; i < results.size(); i++) {
            std::cout << "Neighbor " << i << ": " << results[i] << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}