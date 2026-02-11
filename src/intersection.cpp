#include "intersection.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

IntersectionDetector::IntersectionDetector(CryptoEngine* eng, GeometryEngine* geo) 
    : engine(eng), geometry(geo) {
    if (!engine || !engine->isInitialized()) {
        throw std::runtime_error("IntersectionDetector requires initialized CryptoEngine");
    }
    if (!geometry) {
        throw std::runtime_error("IntersectionDetector requires GeometryEngine");
    }
}

// ===== Détection en clair (référence) =====
std::vector<IntersectionDetector::IntersectionResult> 
IntersectionDetector::detectIntersectionsClear(const Path& path1, const Path& path2) {
    std::vector<IntersectionResult> results;
    
    // Tester chaque paire de segments
    for (size_t i = 0; i < path1.size() - 1; i++) {
        Segment seg1 = {path1[i], path1[i + 1]};
        
        for (size_t j = 0; j < path2.size() - 1; j++) {
            Segment seg2 = {path2[j], path2[j + 1]};
            
            bool intersects = GeometryEngine::doSegmentsIntersectClear(seg1, seg2);
            
            if (intersects) {
                results.emplace_back(i, j, true);
                intersectionsFound++;
            }
            
            totalTests++;
        }
    }
    
    return results;
}

// ===== Détection chiffrée =====
CryptoEngine::CiphertextCKKS 
IntersectionDetector::testSegmentIntersectionEncrypted(
    const Segment& seg1, const Segment& seg2) {
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Utiliser directement la fonction de GeometryEngine qui fait tout
    auto ct_result = geometry->checkSegmentIntersection(seg1, seg2);
    
    auto end = std::chrono::high_resolution_clock::now();
    totalTime += end - start;
    totalTests++;
    
    return ct_result;
}

CryptoEngine::CiphertextCKKS 
IntersectionDetector::combineOrientationsForIntersection(
    const CiphertextCKKS& s1, const CiphertextCKKS& s2,
    const CiphertextCKKS& s3, const CiphertextCKKS& s4) {
    
    // Logique correcte : (s1 XOR s2) AND (s3 XOR s4)
    // XOR(a,b) = a + b - 2*a*b pour bits 0/1
    
    // XOR entre s1 et s2
    auto s1_plus_s2 = engine->add(s1, s2);
    auto s1_times_s2 = engine->mult(s1, s2);
    auto two_s1_s2 = engine->add(s1_times_s2, s1_times_s2);
    auto xor12 = engine->sub(s1_plus_s2, two_s1_s2);
    
    // XOR entre s3 et s4
    auto s3_plus_s4 = engine->add(s3, s4);
    auto s3_times_s4 = engine->mult(s3, s4);
    auto two_s3_s4 = engine->add(s3_times_s4, s3_times_s4);
    auto xor34 = engine->sub(s3_plus_s4, two_s3_s4);
    
    // AND des deux XOR
    auto result = engine->mult(xor12, xor34);
    
    return result;
}

std::vector<CryptoEngine::CiphertextCKKS> 
IntersectionDetector::detectIntersectionsEncrypted(
    const Path& path1, const Path& path2) {
    
    std::vector<CiphertextCKKS> results;
    
    for (size_t i = 0; i < path1.size() - 1; i++) {
        Segment seg1 = {path1[i], path1[i + 1]};
        
        for (size_t j = 0; j < path2.size() - 1; j++) {
            Segment seg2 = {path2[j], path2[j + 1]};
            
            auto ct_result = testSegmentIntersectionEncrypted(seg1, seg2);
            results.push_back(ct_result);
        }
    }
    
    return results;
}

std::vector<IntersectionDetector::IntersectionResult> 
IntersectionDetector::detectIntersectionsHybrid(
    const Path& path1, const Path& path2) {
    
    std::vector<IntersectionResult> results;
    
    std::cout << "Testing " << (path1.size()-1) << " x " << (path2.size()-1) 
              << " segment pairs..." << std::endl;
    
    for (size_t i = 0; i < path1.size() - 1; i++) {
        Segment seg1 = {path1[i], path1[i + 1]};
        
        for (size_t j = 0; j < path2.size() - 1; j++) {
            Segment seg2 = {path2[j], path2[j + 1]};
            
            // Test chiffré
            auto ct_result = testSegmentIntersectionEncrypted(seg1, seg2);
            
            // Déchiffrer pour analyse
            double confidence = engine->decryptValue(ct_result);
            
            // Décision binaire
            bool intersects = (confidence > 0.5);
            
            if (intersects) {
                intersectionsFound++;
                std::cout << "  Intersection found: seg[" << i << "] x seg[" << j 
                          << "] (confidence: " << confidence << ")" << std::endl;
            }
            
            results.emplace_back(i, j, intersects, confidence);
        }
    }
    
    return results;
}

// ===== Optimisations (désactivées pour l'instant) =====
bool IntersectionDetector::boundingBoxesIntersect(const Segment&, const Segment&) {
    // Désactivé pour tester le protocole pur
    return true;
}

std::vector<CryptoEngine::CiphertextCKKS> 
IntersectionDetector::processSegmentBatch(
    const std::vector<std::pair<Segment, Segment>>& pairs) {
    
    std::vector<CiphertextCKKS> results;
    results.reserve(pairs.size());
    
    for (const auto& [seg1, seg2] : pairs) {
        results.push_back(testSegmentIntersectionEncrypted(seg1, seg2));
    }
    
    return results;
}

// ===== Statistiques =====
void IntersectionDetector::printStatistics() const {
    std::cout << "\n=== Intersection Detector Statistics ===" << std::endl;
    std::cout << "Total tests performed: " << totalTests << std::endl;
    std::cout << "Intersections found: " << intersectionsFound << std::endl;
    
    if (totalTests > 0) {
        double hitRate = (100.0 * intersectionsFound) / totalTests;
        std::cout << "Hit rate: " << hitRate << "%" << std::endl;
        
        double avgTime = totalTime.count() / totalTests;
        std::cout << "Average time per test: " << avgTime * 1000 << " ms" << std::endl;
    }
}

void IntersectionDetector::resetStatistics() {
    totalTests = 0;
    intersectionsFound = 0;
    totalTime = std::chrono::duration<double>::zero();
}

double IntersectionDetector::getAverageIntersectionTime() const {
    if (totalTests == 0) return 0.0;
    return totalTime.count() / totalTests;
}