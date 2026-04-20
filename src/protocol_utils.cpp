#include "protocol_utils.hpp"
#include <iostream>
#include <stdexcept>

std::vector<Segment> extractThreeSegments(const Path& path, size_t startIndex) {
    if (path.size() < startIndex + 4) {
        throw std::runtime_error("Not enough points to extract 3 segments");
    }

    return {
        {path[startIndex],     path[startIndex + 1]},
        {path[startIndex + 1], path[startIndex + 2]},
        {path[startIndex + 2], path[startIndex + 3]}
    };
}

void printSegments(const std::vector<Segment>& segs, const std::string& label) {
    std::cout << "\n" << label << ":\n";
    for (size_t i = 0; i < segs.size(); ++i) {
        const auto& s = segs[i];
        std::cout << "  S" << i << " = [("
                  << s.first.x << "," << s.first.y << "," << s.first.z << ") -> ("
                  << s.second.x << "," << s.second.y << "," << s.second.z << ")]\n";
    }
}