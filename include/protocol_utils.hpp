#ifndef PROTOCOL_UTILS_HPP
#define PROTOCOL_UTILS_HPP

#include "types.hpp"
#include <string>
#include <vector>

std::vector<Segment> extractThreeSegments(const Path& path, size_t startIndex);
void printSegments(const std::vector<Segment>& segs, const std::string& label);

#endif