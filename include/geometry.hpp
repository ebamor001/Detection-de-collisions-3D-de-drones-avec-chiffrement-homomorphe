#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include "engine.hpp"
#include "types.hpp"
#include <vector>
#include <tuple>
#include <utility>

class GeometryEngine {
public:
    using CiphertextCKKS = CryptoEngine::CiphertextCKKS;

    enum Orientation {
        COLLINEAR = 0,
        CLOCKWISE = 1,
        COUNTERCLOCKWISE = 2
    };

    enum DropAxis { DROP_X=0, DROP_Y=1, DROP_Z=2 };

    explicit GeometryEngine(CryptoEngine* engine);


    // ===== 1) Fonctions en clair =====
    static Orientation orientationClear2D(const IntPoint& p, const IntPoint& q, const IntPoint& r, DropAxis drop);
    static bool onSegmentClear2D(const IntPoint& p, const IntPoint& q, const IntPoint& r, DropAxis drop);
    static bool doSegmentsIntersectClear2D(const Segment& s1, const Segment& s2, DropAxis drop);
    static bool doSegmentsIntersectClear3D(const Segment& seg1, const Segment& seg2); // <- 3D wrapper clair (projection + coplanarité)

    // ===== 2) HE primitives: orientation (chiffrés) =====
    
    CiphertextCKKS computeOrientationValue2D(const IntPoint& p, const IntPoint& q, const IntPoint& r, DropAxis drop);
    CiphertextCKKS computeOrientation2D(const IntPoint& p, const IntPoint& q, const IntPoint& r, DropAxis drop);    
    CiphertextCKKS extractOrientationSign(const CiphertextCKKS& orientationVal);
    CiphertextCKKS onSegment2D(const IntPoint& p, const IntPoint& q, const IntPoint& r,DropAxis drop, double eps);
    CiphertextCKKS checkSegmentIntersection2D(const Segment& seg1, const Segment& seg2, DropAxis drop);
    CiphertextCKKS checkSegmentIntersection3D(const Segment& seg1, const Segment& seg2);
// nouveau : version batchee (N paires en parallele)
    std::vector<double> batchCheckIntersection3D(
        const Segment& mySeg,
        const std::vector<Segment>& neighbors);


    // ===== 3) Validation / stats =====
    bool validatePoints(const IntPoint& p, const IntPoint& q, const IntPoint& r) const;
    void printStats() const;
        
    void resetBootstrapCount() { bootstrapCount = 0; }    
    size_t getOrientationComputations() const { return orientationComputations; }
    size_t getSignExtractions() const { return signExtractions; }
    size_t getIntersectionTests() const { return intersectionTests; }
    size_t getBootstrapCount() const { return bootstrapCount; }

private:
    CryptoEngine* engine;
    
    mutable size_t orientationComputations = 0;
    mutable size_t signExtractions = 0;
    mutable size_t intersectionTests = 0;
    mutable size_t bootstrapCount = 0;

};

#endif // GEOMETRY_HPP
