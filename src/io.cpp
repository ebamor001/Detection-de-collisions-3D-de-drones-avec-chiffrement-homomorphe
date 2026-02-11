#include "io.hpp"
#include <regex>
#include <iostream>
#include <sstream>
#include <random>
#include <climits>  // Pour LONG_MAX
#include <stdexcept>

Path PathIO::parse_line(const std::string& line) {
    Path path;
    
    // Regex pour extraire les paires (x,y)
    // Supporte les nombres négatifs
    std::regex point_regex(R"(\((-?\d+),(-?\d+)\))");
    
    // Itérateur sur tous les matches
    auto points_begin = std::sregex_iterator(line.begin(), line.end(), point_regex);
    auto points_end = std::sregex_iterator();
    
    // Extraire chaque point
    for (std::sregex_iterator it = points_begin; it != points_end; ++it) {
        std::smatch match = *it;
        
        // match[0] = match complet "(x,y)"
        // match[1] = x
        // match[2] = y
        long x = std::stol(match[1].str());
        long y = std::stol(match[2].str());
        
        path.emplace_back(x, y);
    }
    
    if (path.empty()) {
        std::cerr << "Warning: No points found in line: " << line << std::endl;
    }
    
    return path;
}

Path PathIO::read_single_path(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }
    
    std::string line;
    std::getline(file, line);
    file.close();
    
    Path path = parse_line(line);
    
    if (path.empty()) {
        throw std::runtime_error("No valid path found in file: " + filename);
    }
    
    // Valider le chemin
    if (!validate_path(path)) {
        throw std::runtime_error("Path validation failed for file: " + filename);
    }
    
    return path;
}

std::vector<Path> PathIO::read_multiple_paths(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }
    
    std::vector<Path> paths;
    std::string line;
    
    while (std::getline(file, line)) {
        // Ignorer les lignes vides
        if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }
        
        Path path = parse_line(line);
        if (!path.empty() && validate_path(path)) {
            paths.push_back(path);
        }
    }
    
    file.close();
    return paths;
}

void PathIO::write_path(const Path& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot create file: " + filename);
    }
    
    for (const auto& point : path) {
        file << "(" << point.x << "," << point.y << ")";
    }
    file << std::endl;
    file.close();
}

bool PathIO::validate_path(const Path& path) {
    if (path.size() < 2) {
        std::cerr << "Path validation failed: need at least 2 points" << std::endl;
        return false;
    }
    
    for (size_t i = 0; i < path.size(); ++i) {
        if (!is_point_valid(path[i])) {
            std::cerr << "Path validation failed: point " << i 
                      << " " << path[i] << " out of bounds" << std::endl;
            return false;
        }
        
        // Vérifier qu'il n'y a pas de points consécutifs identiques
        if (i > 0 && path[i] == path[i-1]) {
            std::cerr << "Path validation failed: duplicate consecutive points at " 
                      << i-1 << " and " << i << std::endl;
            return false;
        }
    }
    
    return true;
}

Path PathIO::generate_random_path(size_t num_points, unsigned seed) {
    if (num_points < 2) {
        throw std::invalid_argument("Need at least 2 points for a path");
    }
    
    std::mt19937 gen(seed);
    std::uniform_int_distribution<long> dist(
        DroneConstants::MIN_COORDINATE, 
        DroneConstants::MAX_COORDINATE
    );
    
    Path path;
    IntPoint last_point(LONG_MAX, LONG_MAX); // Point impossible
    
    while (path.size() < num_points) {
        IntPoint new_point(dist(gen), dist(gen));
        
        // Éviter les points consécutifs identiques
        if (new_point != last_point) {
            path.push_back(new_point);
            last_point = new_point;
        }
    }
    
    return path;
}

void PathIO::print_path(const Path& path, const std::string& name) {
    std::cout << name << " (" << path.size() << " points): ";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i < path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
}

bool PathIO::is_point_valid(const IntPoint& p) {
    return p.x >= DroneConstants::MIN_COORDINATE && 
           p.x <= DroneConstants::MAX_COORDINATE &&
           p.y >= DroneConstants::MIN_COORDINATE && 
           p.y <= DroneConstants::MAX_COORDINATE;
}
