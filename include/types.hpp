#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <utility>
#include <string>
#include <iostream>

/**
 * Structure représentant un point avec coordonnées entières
 * Bornes: -99 <= x,y <= 99 (comme dans le papier original)
 */
struct IntPoint {
    long x;
    long y;
    long z;
    
    IntPoint() : x(0), y(0) , z(0) {}
    IntPoint(long x_, long y_, long z_) : x(x_), y(y_), z(z_) {}
    
    // Opérateurs utiles
    bool operator==(const IntPoint& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    bool operator!=(const IntPoint& other) const {
        return !(*this == other);
    }
    
    // Pour debug
    friend std::ostream& operator<<(std::ostream& os, const IntPoint& p) {
        os << "(" << p.x << "," << p.y << "," << p.z << ")";
        return os;
    }
};

// Type pour un chemin (suite de points)
using Path = std::vector<IntPoint>;

// Type pour un segment (paire de points)
using Segment = std::pair<IntPoint, IntPoint>;

// Structure pour stocker le résultat d'une détection de collision
struct CollisionResult {
    size_t alice_segment_index;
    size_t bob_segment_index;
    bool has_collision;
    
    CollisionResult(size_t a_idx, size_t b_idx, bool collision) 
        : alice_segment_index(a_idx), bob_segment_index(b_idx), has_collision(collision) {}
};

// Constantes du système (cohérentes avec le papier)
namespace DroneConstants {
    constexpr long MAX_COORDINATE = 100;
    constexpr long MIN_COORDINATE = -99;
    constexpr size_t DEFAULT_BATCH_SIZE = 8;  // Pour vectorisation CKKS
}

#endif // TYPES_HPP