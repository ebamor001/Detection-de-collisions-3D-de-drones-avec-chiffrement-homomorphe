#ifndef PROTOCOL_TYPES_HPP
#define PROTOCOL_TYPES_HPP

#include "engine.hpp"
#include "types.hpp"
#include <vector>
#include <string>

struct EncPoint2D {
    CryptoEngine::CiphertextCKKS x;
    CryptoEngine::CiphertextCKKS y;
};

struct EncSegment2D {
    EncPoint2D p1;
    EncPoint2D p2;
};

struct BobPublicBundle {
    std::vector<EncSegment2D> encryptedSegments; // taille 3
};

struct AliceResultBundle {
    std::vector<CryptoEngine::CiphertextCKKS> encryptedRiskPerBobSegment; // taille 3
};

#endif