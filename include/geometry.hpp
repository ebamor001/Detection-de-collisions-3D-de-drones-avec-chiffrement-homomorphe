#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include "engine.hpp"
#include "types.hpp"
#include <vector>
#include <tuple>

class GeometryEngine {
public:
    using CiphertextCKKS = CryptoEngine::CiphertextCKKS;

    enum Orientation {
        COLLINEAR = 0,
        CLOCKWISE = 1,
        COUNTERCLOCKWISE = 2
    };

    explicit GeometryEngine(CryptoEngine* engine);
    
    // Fonctions en clair (static)
    static Orientation orientationClear(const IntPoint& p, const IntPoint& q, const IntPoint& r);
    static bool onSegmentClear(const IntPoint& p, const IntPoint& q, const IntPoint& r);
    static bool doSegmentsIntersectClear(const Segment& seg1, const Segment& seg2);
    
    // Fonctions chiffrées - scalaires
    CiphertextCKKS computeOrientationValue(const IntPoint& p, const IntPoint& q, const IntPoint& r);
    CiphertextCKKS extractOrientationSign(const CiphertextCKKS& orientationVal);
    CiphertextCKKS computeOrientation(const IntPoint& p, const IntPoint& q, const IntPoint& r);
    CiphertextCKKS onSegmentEncrypted(const IntPoint& p, const IntPoint& q, const IntPoint& r);
    
    // Fonctions chiffrées - packées
    CiphertextCKKS computeOrientationValuesPacked4(const Segment& s1, const Segment& s2);
    CiphertextCKKS checkSegmentIntersection(const Segment& seg1, const Segment& seg2);
    CiphertextCKKS checkSegmentIntersectionFast(const Segment& seg1, const Segment& seg2);
    
    // Fonctions chiffrées - batched
    CiphertextCKKS computeOrientationsBatched(
        const std::vector<std::tuple<IntPoint, IntPoint, IntPoint>>& triplets);
    
    std::vector<CiphertextCKKS> computeOrientationsBatch(
        const std::vector<std::tuple<IntPoint, IntPoint, IntPoint>>& triplets);
    
    // Validation et stats
    bool validatePoints(const IntPoint& p, const IntPoint& q, const IntPoint& r) const;
    void printStats() const;
    
    size_t getOrientationComputations() const { return orientationComputations; }
    size_t getSignExtractions() const { return signExtractions; }
    size_t getIntersectionTests() const { return intersectionTests; }
    size_t getBootstrapCount() const { return bootstrapCount; }


    // Test on-segment entièrement chiffré
    CiphertextCKKS onSegmentHE(const IntPoint& p, const IntPoint& q, 
                            const IntPoint& r, double tauBox);
    // On-segment par projection et colinéarité
    CiphertextCKKS onSegmentHE_full(const IntPoint& p, const IntPoint& q, 
                                    const IntPoint& r, double tauOri, double epsProj);
    // Dans la section public
    void resetBootstrapCount() { bootstrapCount = 0; }

    // Version optimisée
    CiphertextCKKS checkSegmentIntersectionOptimized(const Segment& seg1, const Segment& seg2);
    CiphertextCKKS checkSegmentIntersectionBasic(const Segment& seg1, const Segment& seg2);

    // Version 3D
    CiphertextCKKS checkSegmentIntersection3D(const Segment& seg1, const Segment& seg2);
    CiphertextCKKS checkCollision3D(const Segment& seg1, const Segment& seg2);

private:
    CryptoEngine* engine;
    
    mutable size_t orientationComputations = 0;
    mutable size_t signExtractions = 0;
    mutable size_t intersectionTests = 0;
    mutable size_t bootstrapCount = 0;

    
    // Helper pour on-segment optimisé
    CiphertextCKKS onSegmentOptimized(const IntPoint& p, const IntPoint& q, 
                                  const IntPoint& r, double eps);   
};

#endif // GEOMETRY_HPP