#ifndef INTERSECTION_HPP
#define INTERSECTION_HPP

#include "types.hpp"
#include "engine.hpp"
#include "geometry.hpp"
#include <vector>
#include <chrono>

/**
 * Module de détection d'intersection pour chemins de drones
 * Implémente l'algorithme complet du papier avec les 4 orientations
 */
class IntersectionDetector {
public:
    using CiphertextCKKS = CryptoEngine::CiphertextCKKS;
    
    struct IntersectionResult {
        size_t seg1_index;
        size_t seg2_index;
        bool intersects;
        double confidence;  // Pour la version chiffrée (proche de 1.0 = intersection)
        
        IntersectionResult(size_t s1, size_t s2, bool inter, double conf = 1.0)
            : seg1_index(s1), seg2_index(s2), intersects(inter), confidence(conf) {}
    };
    
    IntersectionDetector(CryptoEngine* engine, GeometryEngine* geometry);
    
    // ===== Détection complète en clair =====
    
    /**
     * Détecte toutes les intersections entre deux chemins (clair)
     */
    std::vector<IntersectionResult> detectIntersectionsClear(
        const Path& path1, 
        const Path& path2
    );
    
    // ===== Détection chiffrée =====
    
    /**
     * Teste l'intersection de deux segments (version chiffrée complète)
     * Implémente l'algorithme avec 4 orientations
     */
    CiphertextCKKS testSegmentIntersectionEncrypted(
        const Segment& seg1, 
        const Segment& seg2
    );
    
    /**
     * Détecte les intersections entre deux chemins (version chiffrée)
     * Retourne un vecteur de valeurs chiffrées (~1.0 si intersection)
     */
    std::vector<CiphertextCKKS> detectIntersectionsEncrypted(
        const Path& path1, 
        const Path& path2
    );
    
    /**
     * Version hybride : calculs chiffrés mais résultats déchiffrés
     * Plus pratique pour l'analyse
     */
    std::vector<IntersectionResult> detectIntersectionsHybrid(
        const Path& path1, 
        const Path& path2
    );
    
    // ===== Optimisations =====
    
    /**
     * Pré-filtre les segments par boîtes englobantes
     * Évite les calculs inutiles
     */
    static bool boundingBoxesIntersect(const Segment& seg1, const Segment& seg2);
    
    /**
     * Traitement par batch de plusieurs paires de segments
     */
    std::vector<CiphertextCKKS> processSegmentBatch(
        const std::vector<std::pair<Segment, Segment>>& pairs
    );
    
    // ===== Analyse et stats =====
    
    void printStatistics() const;
    void resetStatistics();
    
    double getAverageIntersectionTime() const;
    size_t getTotalIntersectionTests() const { return totalTests; }
    size_t getTotalIntersectionsFound() const { return intersectionsFound; }
    
private:
    CryptoEngine* engine;
    GeometryEngine* geometry;
    
    // Statistiques
    size_t totalTests = 0;
    size_t intersectionsFound = 0;
    std::chrono::duration<double> totalTime{0};
    
    // Helper pour combiner les orientations en logique d'intersection
    CiphertextCKKS combineOrientationsForIntersection(
        const CiphertextCKKS& o1,
        const CiphertextCKKS& o2,
        const CiphertextCKKS& o3,
        const CiphertextCKKS& o4
    );
};

#endif // INTERSECTION_HPP
