#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <utility>
#include <string>
#include <iostream>
#include <random>

#define BATCH_SIZE 32

/**
 * Structure représentant un point avec coordonnées entières
 * Bornes: -99 <= x,y <= 99 (comme dans le papier original)
 */
struct IntPoint {
    double x;
    double y;
    double z;
    
    IntPoint() : x(0), y(0) , z(0) {}
    IntPoint(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    // Méthodes
    std::vector<double> PointToVector(){
        return {x,y,z};
    }

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
// Point 3D pour les trajectoires de drones
struct Point3D {
    double x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};
// Structure représentant un drone
struct Drone{
    int id;
    int NbSegments;
    std::vector<Segment> segments = {};
    std::vector<double> points = {};
    double z;

    std::vector<int> id_alice = {};
    std::vector<int> id_bob = {};

    Drone() : id(0), z(0) {}

    Drone(int id_, std::vector<double>points_) : id(id_), points(points_) {
        NbSegments = (points.size() / 3) - 1;
        z = points[2];
        GenerateSegments();
    }

    Drone(int id_, double z_, int NbSegments_) : id(id_), z(z_), NbSegments(NbSegments_){
        GeneratePoints();
        GenerateSegments();
    }

    void GeneratePoints(){
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-99.0, 100.0);
        points.clear();
        int max_points = (1 + NbSegments) * 3;
        for(int i=1; i<max_points+1; i++){
            if(i % 3 == 0){ points.push_back(z); continue; }
            points.push_back(dis(gen));
        }
    }

    void GenerateSegments(){
        segments.clear();
        if (points.size() < 6) return;
        auto point1 = IntPoint(points[0], points[1], points[2]);
        for(int i=0; i<NbSegments;i++){
            int j = 3*(i+1);
            auto point2 = IntPoint(points[j], points[j+1], points[j+2]);
            segments.push_back(std::make_pair(point1, point2));
            point1 = point2;
        }
    }

    void displaySegment(int i) {
        std::cout << "Drone " << id << ": " << segments[i].first << " to " << segments[i].second << std::endl;
    }

    friend std::ostream& operator<<(std::ostream& os, Drone& drone){
        os << "id : " << drone.id;
        return os;
    }
};

// Trajectoire 3D
using Trajectory3D = std::vector<Point3D>;
#endif // TYPES_HPP