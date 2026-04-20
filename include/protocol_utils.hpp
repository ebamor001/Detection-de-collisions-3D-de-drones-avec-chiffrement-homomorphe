#ifndef PROTOCOL_UTILS_HPP
#define PROTOCOL_UTILS_HPP
#include "protocol_types.hpp"
#include "types.hpp"
#include <vector>

std::vector<Segment> extractThreeSegments(const Path& path, size_t startIndex);

EncPoint2D encryptPoint2D(CryptoEngine& engine, const IntPoint& p);
EncSegment2D encryptSegment2D(CryptoEngine& engine, const Segment& s);

#endif