#include "geometry.hpp"
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cmath>

GeometryEngine::GeometryEngine(CryptoEngine *eng) : engine(eng)
{
    if (!engine || !engine->isInitialized())
    {
        throw std::runtime_error("GeometryEngine requires initialized CryptoEngine");
    }
}

// helpers
static GeometryEngine::DropAxis chooseDropAxisFromNormal(long nx, long ny, long nz)
{
    long ax = std::llabs(nx), ay = std::llabs(ny), az = std::llabs(nz);
    if (ax >= ay && ax >= az)
        return GeometryEngine::DROP_X;
    if (ay >= ax && ay >= az)
        return GeometryEngine::DROP_Y;
    return GeometryEngine::DROP_Z;
}

static std::pair<long, long> project2D(const IntPoint &p, GeometryEngine::DropAxis drop)
{
    if (drop == GeometryEngine::DROP_X)
        return {p.y, p.z};
    if (drop == GeometryEngine::DROP_Y)
        return {p.x, p.z};
    return {p.x, p.y};
}

// ===== helpers ciphertext slots =====

static Plaintext makeSlotMaskPlain(
    const CryptoContext<DCRTPoly>& cc,
    uint32_t slotCount,
    uint32_t slotIndex)
{
    std::vector<double> mask(slotCount, 0.0);
    if (slotIndex < slotCount)
        mask[slotIndex] = 1.0;
    return cc->MakeCKKSPackedPlaintext(mask);
}

static CryptoEngine::CiphertextCKKS keepOnlySlot(
    CryptoEngine* engine,
    const CryptoEngine::CiphertextCKKS& ct,
    uint32_t slotIndex)
{
    auto cc = engine->getCKKSContext();
    auto ptMask = makeSlotMaskPlain(cc, engine->getSlotCount(), slotIndex);
    return cc->EvalMult(ct, ptMask);
}

static CryptoEngine::CiphertextCKKS keepSlot0(
    CryptoEngine* engine,
    const CryptoEngine::CiphertextCKKS& ct)
{
    return keepOnlySlot(engine, ct, 0);
}

static CryptoEngine::CiphertextCKKS extractSlotTo0(
    CryptoEngine* engine,
    const CryptoEngine::CiphertextCKKS& ct,
    uint32_t slotIndex)
{
    auto cc = engine->getCKKSContext();
    auto rotated = (slotIndex == 0) ? ct : cc->EvalRotate(ct, -static_cast<int32_t>(slotIndex));
    return keepSlot0(engine, rotated);
}

static CryptoEngine::CiphertextCKKS gateBit(
    CryptoEngine* engine,
    const CryptoEngine::CiphertextCKKS& bit)
{
    return keepSlot0(engine, bit);
}

// ===== 1) Fonctions en clair =====
GeometryEngine::Orientation GeometryEngine::orientationClear2D(
    const IntPoint &p, const IntPoint &q, const IntPoint &r, DropAxis drop)
{
    auto [px, py] = project2D(p, drop);
    auto [qx, qy] = project2D(q, drop);
    auto [rx, ry] = project2D(r, drop);

    long val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy);

    if (val == 0)
        return COLLINEAR;
    return (val > 0) ? CLOCKWISE : COUNTERCLOCKWISE;
}

bool GeometryEngine::onSegmentClear2D(
    const IntPoint &p, const IntPoint &q, const IntPoint &r, DropAxis drop)
{
    auto [px, py] = project2D(p, drop);
    auto [qx, qy] = project2D(q, drop);
    auto [rx, ry] = project2D(r, drop);

    return (qx <= std::max(px, rx) && qx >= std::min(px, rx) &&
            qy <= std::max(py, ry) && qy >= std::min(py, ry));
}

bool GeometryEngine::doSegmentsIntersectClear2D(
    const Segment &s1, const Segment &s2, DropAxis drop)
{
    const auto &p1 = s1.first;
    const auto &q1 = s1.second;
    const auto &p2 = s2.first;
    const auto &q2 = s2.second;

    auto o1 = orientationClear2D(p1, q1, p2, drop);
    auto o2 = orientationClear2D(p1, q1, q2, drop);
    auto o3 = orientationClear2D(p2, q2, p1, drop);
    auto o4 = orientationClear2D(p2, q2, q1, drop);

    if (o1 != o2 && o3 != o4)
        return true;

    if (o1 == COLLINEAR && onSegmentClear2D(p1, p2, q1, drop))
        return true;
    if (o2 == COLLINEAR && onSegmentClear2D(p1, q2, q1, drop))
        return true;
    if (o3 == COLLINEAR && onSegmentClear2D(p2, p1, q2, drop))
        return true;
    if (o4 == COLLINEAR && onSegmentClear2D(p2, q1, q2, drop))
        return true;

    return false;
}

bool GeometryEngine::doSegmentsIntersectClear3D(const Segment &s1, const Segment &s2)
{
    const auto &p1 = s1.first;
    const auto &q1 = s1.second;
    const auto &p2 = s2.first;
    const auto &q2 = s2.second;

    long d1x = q1.x - p1.x, d1y = q1.y - p1.y, d1z = q1.z - p1.z;
    long d2x = q2.x - p2.x, d2y = q2.y - p2.y, d2z = q2.z - p2.z;

    long nx = d1y * d2z - d1z * d2y;
    long ny = d1z * d2x - d1x * d2z;
    long nz = d1x * d2y - d1y * d2x;
    if (nx == 0 && ny == 0 && nz == 0)
    {
        return doSegmentsIntersectClear2D(s1, s2, GeometryEngine::DROP_Z);
    }

    long wx = p2.x - p1.x, wy = p2.y - p1.y, wz = p2.z - p1.z;
    long cop = wx * nx + wy * ny + wz * nz;
    if (cop != 0)
        return false;

    DropAxis drop = chooseDropAxisFromNormal(nx, ny, nz);
    return doSegmentsIntersectClear2D(s1, s2, drop);
}

// ===== 2) HE primitives: orientation/intersection (chiffrés) =====
CryptoEngine::CiphertextCKKS GeometryEngine::computeOrientationValue2D(
    const IntPoint &p, const IntPoint &q, const IntPoint &r, DropAxis drop)
{
    auto [px, py] = project2D(p, drop);
    auto [qx, qy] = project2D(q, drop);
    auto [rx, ry] = project2D(r, drop);

    double dy1 = double(qy - py);
    double dx2 = double(rx - qx);
    double dx1 = double(qx - px);
    double dy2 = double(ry - qy);

    auto ct_dy1 = engine->encryptValue(dy1);
    auto ct_dx2 = engine->encryptValue(dx2);
    auto ct_dx1 = engine->encryptValue(dx1);
    auto ct_dy2 = engine->encryptValue(dy2);

    auto ct_val = engine->sub(engine->mult(ct_dy1, ct_dx2),
                              engine->mult(ct_dx1, ct_dy2));
    orientationComputations++;
    return ct_val;
}

CryptoEngine::CiphertextCKKS GeometryEngine::computeOrientation2D(
    const IntPoint &p, const IntPoint &q, const IntPoint &r, DropAxis drop)
{
    auto ct_val = computeOrientationValue2D(p, q, r, drop);
    if (engine->isSwitchingReady())
        return extractOrientationSign(ct_val);
    return ct_val;
}

CryptoEngine::CiphertextCKKS GeometryEngine::extractOrientationSign(
    const CiphertextCKKS &orientationVal)
{
    if (!engine->isSwitchingReady())
    {
        throw std::runtime_error("Scheme switching not ready");
    }

    auto ct_sign = engine->compareGreaterThanZero(orientationVal);
    signExtractions++;
    return ct_sign;
}

CryptoEngine::CiphertextCKKS GeometryEngine::onSegment2D(
    const IntPoint &p, const IntPoint &q, const IntPoint &r,
    DropAxis drop, double eps)
{
    auto [px, py] = project2D(p, drop);
    auto [qx, qy] = project2D(q, drop);
    auto [rx, ry] = project2D(r, drop);

    auto cpx = engine->encryptValue((double)px);
    auto cpy = engine->encryptValue((double)py);
    auto cqx = engine->encryptValue((double)qx);
    auto cqy = engine->encryptValue((double)qy);
    auto crx = engine->encryptValue((double)rx);
    auto cry = engine->encryptValue((double)ry);

    auto ux = engine->sub(crx, cpx);
    auto uy = engine->sub(cry, cpy);
    auto vx = engine->sub(cqx, cpx);
    auto vy = engine->sub(cqy, cpy);

    auto s = engine->add(engine->mult(vx, ux), engine->mult(vy, uy));
    auto uu = engine->add(engine->mult(ux, ux), engine->mult(uy, uy));

    auto epsS = engine->constLike(s, eps);
    auto epsU = engine->constLike(uu, eps);

    auto term1 = engine->add(s, epsS);
    auto term2 = engine->sub(engine->add(uu, epsU), s);
    auto prod = engine->mult(term1, term2);

    auto one = engine->oneLike(prod);
    auto onSeg = engine->sub(one, engine->ltZero(prod));

    bootstrapCount++;
    return onSeg;
}

CryptoEngine::CiphertextCKKS GeometryEngine::checkSegmentIntersection2D(
    const Segment &seg1, const Segment &seg2, DropAxis drop)
{
    const double tauOri = 1e-5;
    const double epsProj = 5e-6;
    auto cc = engine->getCKKSContext();

    const auto &p1 = seg1.first;
    const auto &q1 = seg1.second;
    const auto &p2 = seg2.first;
    const auto &q2 = seg2.second;

    auto o1 = computeOrientationValue2D(p1, q1, p2, drop);
    auto o2 = computeOrientationValue2D(p1, q1, q2, drop);
    auto o3 = computeOrientationValue2D(p2, q2, p1, drop);
    auto o4 = computeOrientationValue2D(p2, q2, q1, drop);

    auto opp12 = engine->ltZero(engine->mult(o1, o2));
    auto opp34 = engine->ltZero(engine->mult(o3, o4));
    bootstrapCount += 2;
    auto generalBit = engine->eAnd(opp12, opp34);

    double gVal = engine->decryptValue(generalBit);
    if (gVal > 0.5)
    {
        intersectionTests++;
        return generalBit;
    }

    auto o1sq = engine->mult(o1, o1);
    auto o2sq = engine->mult(o2, o2);
    auto o3sq = engine->mult(o3, o3);
    auto o4sq = engine->mult(o4, o4);

    auto tau1 = engine->constLike(o1sq, tauOri * tauOri);
    auto tau2 = engine->constLike(o2sq, tauOri * tauOri);
    auto tau3 = engine->constLike(o3sq, tauOri * tauOri);
    auto tau4 = engine->constLike(o4sq, tauOri * tauOri);

    auto z1 = cc->EvalSub(engine->oneLike(o1sq),
                          engine->compareGreaterThanZero(cc->EvalSub(o1sq, tau1)));
    auto z2 = cc->EvalSub(engine->oneLike(o2sq),
                          engine->compareGreaterThanZero(cc->EvalSub(o2sq, tau2)));
    auto z3 = cc->EvalSub(engine->oneLike(o3sq),
                          engine->compareGreaterThanZero(cc->EvalSub(o3sq, tau3)));
    auto z4 = cc->EvalSub(engine->oneLike(o4sq),
                          engine->compareGreaterThanZero(cc->EvalSub(o4sq, tau4)));
    bootstrapCount += 4;

    auto z_any = engine->eOr(engine->eOr(z1, z2), engine->eOr(z3, z4));

    double zanyVal = engine->decryptValue(z_any);
    if (zanyVal < 0.5)
    {
        intersectionTests++;
        return engine->constLike(z_any, 0.0);
    }

    auto on1 = onSegment2D(p1, p2, q1, drop, epsProj);
    auto on2 = onSegment2D(p1, q2, q1, drop, epsProj);
    auto on3 = onSegment2D(p2, p1, q2, drop, epsProj);
    auto on4 = onSegment2D(p2, q1, q2, drop, epsProj);

    auto anyCol = engine->eOr(
        engine->eOr(engine->eAnd(z1, on1), engine->eAnd(z2, on2)),
        engine->eOr(engine->eAnd(z3, on3), engine->eAnd(z4, on4)));

    intersectionTests++;
    return engine->eOr(generalBit, anyCol);
}

// Point d'entrée principal
CryptoEngine::CiphertextCKKS GeometryEngine::checkSegmentIntersection3D(
    const Segment &s1, const Segment &s2)
{
    const auto &p1 = s1.first;
    const auto &q1 = s1.second;
    const auto &p2 = s2.first;
    const auto &q2 = s2.second;

    long d1x = q1.x - p1.x, d1y = q1.y - p1.y, d1z = q1.z - p1.z;
    long d2x = q2.x - p2.x, d2y = q2.y - p2.y, d2z = q2.z - p2.z;
    long nx = d1y * d2z - d1z * d2y;
    long ny = d1z * d2x - d1x * d2z;
    long nz = d1x * d2y - d1y * d2x;

    if (nx == 0 && ny == 0 && nz == 0)
    {
        if (p1.z != q1.z || p1.z != p2.z || p1.z != q2.z)
        {
            return engine->encryptValue(0.0);
        }
        return checkSegmentIntersection2D(s1, s2, GeometryEngine::DROP_Z);
    }

    DropAxis drop = chooseDropAxisFromNormal(nx, ny, nz);
    long wx = p2.x - p1.x, wy = p2.y - p1.y, wz = p2.z - p1.z;
    double cop = double(wx * nx + wy * ny + wz * nz);

    const double tauCop = 1e-5;
    auto ct_cop = engine->encryptValue(cop);
    auto copOK = engine->isNearZeroBand(ct_cop, tauCop);
    bootstrapCount += 2;

    auto inter2D = checkSegmentIntersection2D(s1, s2, drop);

    return engine->eAnd(copOK, inter2D);
}

// ===== 2bis) Version full encrypted : entree = 2 ciphertexts =====
CryptoEngine::CiphertextCKKS GeometryEngine::checkSegmentIntersection3DEncrypted(
    const CiphertextCKKS &ctAlice,
    const CiphertextCKKS &ctBob,
    bool sameAltitude)
{
    std::cout << "[geo] start encrypted 3D" << std::endl;

    // --- Extraire les 6 coords de chaque segment vers le slot 0 ---
    auto ax1 = extractSlotTo0(engine, ctAlice, 0);
    auto ay1 = extractSlotTo0(engine, ctAlice, 1);
    auto az1 = extractSlotTo0(engine, ctAlice, 2);
    auto ax2 = extractSlotTo0(engine, ctAlice, 3);
    auto ay2 = extractSlotTo0(engine, ctAlice, 4);
    auto az2 = extractSlotTo0(engine, ctAlice, 5);

    auto bx1 = extractSlotTo0(engine, ctBob, 0);
    auto by1 = extractSlotTo0(engine, ctBob, 1);
    auto bz1 = extractSlotTo0(engine, ctBob, 2);
    auto bx2 = extractSlotTo0(engine, ctBob, 3);
    auto by2 = extractSlotTo0(engine, ctBob, 4);
    auto bz2 = extractSlotTo0(engine, ctBob, 5);

    // --- Directions 3D ---
    auto d1x = engine->sub(ax2, ax1);
    auto d1y = engine->sub(ay2, ay1);
    auto d1z = engine->sub(az2, az1);

    auto d2x = engine->sub(bx2, bx1);
    auto d2y = engine->sub(by2, by1);
    auto d2z = engine->sub(bz2, bz1);

    // --- Normale n = d1 x d2 ---
    auto nx = engine->sub(engine->mult(d1y, d2z), engine->mult(d1z, d2y));
    auto ny = engine->sub(engine->mult(d1z, d2x), engine->mult(d1x, d2z));
    auto nz = engine->sub(engine->mult(d1x, d2y), engine->mult(d1y, d2x));

    auto n2 = engine->add(
        engine->add(engine->mult(nx, nx), engine->mult(ny, ny)),
        engine->mult(nz, nz)
    );

    // n2 = |d1 x d2|^2 >= 0 always, so near-zero = n2 <= tau needs only 1 SS
    const double tauPar = 1e-6;
    auto parallelBit = engine->compareLE(n2, engine->constLike(n2, tauPar));
    auto notParallelBit = engine->eNot(parallelBit);
    bootstrapCount += 1;

    // --- Test coplanarite ---
    auto wx = engine->sub(bx1, ax1);
    auto wy = engine->sub(by1, ay1);
    auto wz = engine->sub(bz1, az1);

    auto cop = engine->add(
        engine->add(engine->mult(wx, nx), engine->mult(wy, ny)),
        engine->mult(wz, nz)
    );

    // cop can be negative, but |cop| < tau <=> cop^2 < tau^2 needs only 1 SS
    const double tauCop = 1e-6;
    auto copOK = engine->isNearZeroSquared(cop, tauCop);
    bootstrapCount += 1;

    // --- Helpers 2D sous chiffrement ---
    auto encOrient2D = [&](const CiphertextCKKS& px, const CiphertextCKKS& py,
                           const CiphertextCKKS& qx, const CiphertextCKKS& qy,
                           const CiphertextCKKS& rx, const CiphertextCKKS& ry) -> CiphertextCKKS
    {
        auto dy1 = engine->sub(qy, py);
        auto dx2 = engine->sub(rx, qx);
        auto dx1 = engine->sub(qx, px);
        auto dy2 = engine->sub(ry, qy);

        return engine->sub(
            engine->mult(dy1, dx2),
            engine->mult(dx1, dy2)
        );
    };

    auto encOnSegment2D = [&](const CiphertextCKKS& px, const CiphertextCKKS& py,
                              const CiphertextCKKS& qx, const CiphertextCKKS& qy,
                              const CiphertextCKKS& rx, const CiphertextCKKS& ry) -> CiphertextCKKS
    {
        auto bx = engine->mult(engine->sub(qx, px), engine->sub(qx, rx));
        auto by = engine->mult(engine->sub(qy, py), engine->sub(qy, ry));

        auto epsx = engine->constLike(bx, 1e-6);
        auto epsy = engine->constLike(by, 1e-6);

        auto inX = engine->compareLE(bx, epsx);
        auto inY = engine->compareLE(by, epsy);
        bootstrapCount += 2;

        return engine->eAnd(inX, inY);
    };

    auto encIntersect2D = [&](const CiphertextCKKS& p1x, const CiphertextCKKS& p1y,
                              const CiphertextCKKS& q1x, const CiphertextCKKS& q1y,
                              const CiphertextCKKS& p2x, const CiphertextCKKS& p2y,
                              const CiphertextCKKS& q2x, const CiphertextCKKS& q2y) -> CiphertextCKKS
    {
        auto o1 = encOrient2D(p1x, p1y, q1x, q1y, p2x, p2y);
        auto o2 = encOrient2D(p1x, p1y, q1x, q1y, q2x, q2y);
        auto o3 = encOrient2D(p2x, p2y, q2x, q2y, p1x, p1y);
        auto o4 = encOrient2D(p2x, p2y, q2x, q2y, q1x, q1y);

        auto opp12 = engine->ltZero(engine->mult(o1, o2));
        auto opp34 = engine->ltZero(engine->mult(o3, o4));
        bootstrapCount += 2;

        auto generalBit = engine->eAnd(opp12, opp34);

        auto z1 = engine->isNearZeroSquared(o1, 1e-6);
        auto z2 = engine->isNearZeroSquared(o2, 1e-6);
        auto z3 = engine->isNearZeroSquared(o3, 1e-6);
        auto z4 = engine->isNearZeroSquared(o4, 1e-6);
        bootstrapCount += 4;

        auto on1 = encOnSegment2D(p1x, p1y, p2x, p2y, q1x, q1y);
        auto on2 = encOnSegment2D(p1x, p1y, q2x, q2y, q1x, q1y);
        auto on3 = encOnSegment2D(p2x, p2y, p1x, p1y, q2x, q2y);
        auto on4 = encOnSegment2D(p2x, p2y, q1x, q1y, q2x, q2y);

        auto c1 = engine->eAnd(z1, on1);
        auto c2 = engine->eAnd(z2, on2);
        auto c3 = engine->eAnd(z3, on3);
        auto c4 = engine->eAnd(z4, on4);

        auto anyCol = engine->eOr(
            engine->eOr(c1, c2),
            engine->eOr(c3, c4)
        );

        return engine->eOr(generalBit, anyCol);
    };

    // --- Projection ---
    // interDropZ (XY plane) is always needed: fast path uses it directly,
    // full path uses it for both axis selection and the parallel case.
    auto interDropZ = encIntersect2D(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2);

    CiphertextCKKS inter2D;
    if (sameAltitude) {
        // Segments are coplanar in XY: skip axis selection and interDropX/Y entirely.
        std::cout << "[geo] same-altitude fast path: drop Z only" << std::endl;
        inter2D = interDropZ;
    } else {
        // Dynamic axis selection via encrypted comparisons.
        auto absnx = engine->mult(nx, nx);
        auto absny = engine->mult(ny, ny);
        auto absnz = engine->mult(nz, nz);

        auto dropX = engine->eAnd(
            engine->compareGE(absnx, absny),
            engine->compareGE(absnx, absnz)
        );

        auto dropY = engine->eAnd(
            engine->compareGE(absny, absnx),
            engine->compareGE(absny, absnz)
        );

        auto dropZ = engine->eAnd(
            engine->eNot(dropX),
            engine->eNot(dropY)
        );
        bootstrapCount += 6;

        auto interDropX = encIntersect2D(ay1, az1, ay2, az2, by1, bz1, by2, bz2);
        auto interDropY = encIntersect2D(ax1, az1, ax2, az2, bx1, bz1, bx2, bz2);

        inter2D = engine->eOr(
            engine->eOr(
                engine->eAnd(dropX, interDropX),
                engine->eAnd(dropY, interDropY)
            ),
            engine->eAnd(dropZ, interDropZ)
        );
    }

    auto nonParallelCase = engine->eAnd(
        notParallelBit,
        engine->eAnd(copOK, inter2D)
    );

    auto parallelCase = engine->eAnd(
        parallelBit,
        engine->eAnd(copOK, interDropZ)
    );

    intersectionTests++;
    std::cout << "[geo] end encrypted 3D" << std::endl;
    std::cout << "[geo] scheme switches approx = " << bootstrapCount << std::endl;
    return engine->eOr(nonParallelCase, parallelCase);
}
// Version batchee de checkSegmentIntersection3D.
std::vector<double> GeometryEngine::batchCheckIntersection3D(
    const Segment &mySeg,
    const std::vector<Segment> &neighbors)
{
    int N = (int)neighbors.size();
    if (N == 0)
        return {};

    const auto &p1 = mySeg.first;
    const auto &q1 = mySeg.second;

    long d1x = q1.x - p1.x, d1y = q1.y - p1.y, d1z = q1.z - p1.z;

    std::vector<double> cops(N);
    std::vector<DropAxis> drops(N);
    std::vector<bool> isParallel(N, false);

    for (int i = 0; i < N; i++)
    {
        const auto &p2 = neighbors[i].first;
        const auto &q2 = neighbors[i].second;

        long d2x = q2.x - p2.x, d2y = q2.y - p2.y, d2z = q2.z - p2.z;

        long nx = d1y * d2z - d1z * d2y;
        long ny = d1z * d2x - d1x * d2z;
        long nz = d1x * d2y - d1y * d2x;

        if (nx == 0 && ny == 0 && nz == 0)
        {
            isParallel[i] = true;
            drops[i] = DROP_Z;
            if (p1.z != q1.z || p1.z != p2.z || p1.z != q2.z)
            {
                cops[i] = 9999.0;
            }
            else
            {
                cops[i] = 0.0;
            }
        }
        else
        {
            long wx = p2.x - p1.x, wy = p2.y - p1.y, wz = p2.z - p1.z;
            cops[i] = double(wx * nx + wy * ny + wz * nz);
            drops[i] = chooseDropAxisFromNormal(nx, ny, nz);
        }
    }

    auto ct_cops = engine->encryptVector(cops);
    auto ct_copOK = engine->isNearZeroBand(ct_cops, 1e-5);
    bootstrapCount += 2;

    auto copResults = engine->decryptVector(ct_copOK, N);

    std::vector<int> groupZ, groupX, groupY;
    for (int i = 0; i < N; i++)
    {
        if (copResults[i] < 0.5)
            continue;
        if (drops[i] == DROP_Z)
            groupZ.push_back(i);
        else if (drops[i] == DROP_X)
            groupX.push_back(i);
        else
            groupY.push_back(i);
    }

    std::vector<double> results(N, 0.0);

    auto processGroup = [&](const std::vector<int> &group, DropAxis drop)
    {
        if (group.empty())
            return;
        int K = (int)group.size();

        auto [p1a, p1b] = project2D(p1, drop);
        auto [q1a, q1b] = project2D(q1, drop);
        double dy1 = double(q1b - p1b);
        double dx1 = double(q1a - p1a);

        std::vector<double> dy1_vec(K, dy1);
        std::vector<double> dx1_vec(K, dx1);

        std::vector<double> dx2_o1(K), dy2_o1(K);
        std::vector<double> dx2_o2(K), dy2_o2(K);
        std::vector<double> dy_seg2(K), dx_seg2(K);
        std::vector<double> dx2_o3(K), dy2_o3(K);
        std::vector<double> dx2_o4(K), dy2_o4(K);

        for (int j = 0; j < K; j++)
        {
            int idx = group[j];
            auto [p2a, p2b] = project2D(neighbors[idx].first, drop);
            auto [q2a, q2b] = project2D(neighbors[idx].second, drop);

            dx2_o1[j] = double(p2a - q1a);
            dy2_o1[j] = double(p2b - q1b);

            dx2_o2[j] = double(q2a - q1a);
            dy2_o2[j] = double(q2b - q1b);

            dy_seg2[j] = double(q2b - p2b);
            dx_seg2[j] = double(q2a - p2a);

            dx2_o3[j] = double(p1a - q2a);
            dy2_o3[j] = double(p1b - q2b);

            dx2_o4[j] = double(q1a - q2a);
            dy2_o4[j] = double(q1b - q2b);
        }

        auto ct_dy1 = engine->encryptVector(dy1_vec);
        auto ct_dx1 = engine->encryptVector(dx1_vec);

        auto ct_o1 = engine->sub(
            engine->mult(ct_dy1, engine->encryptVector(dx2_o1)),
            engine->mult(ct_dx1, engine->encryptVector(dy2_o1)));

        auto ct_o2 = engine->sub(
            engine->mult(ct_dy1, engine->encryptVector(dx2_o2)),
            engine->mult(ct_dx1, engine->encryptVector(dy2_o2)));

        auto ct_dy_s2 = engine->encryptVector(dy_seg2);
        auto ct_dx_s2 = engine->encryptVector(dx_seg2);

        auto ct_o3 = engine->sub(
            engine->mult(ct_dy_s2, engine->encryptVector(dx2_o3)),
            engine->mult(ct_dx_s2, engine->encryptVector(dy2_o3)));

        auto ct_o4 = engine->sub(
            engine->mult(ct_dy_s2, engine->encryptVector(dx2_o4)),
            engine->mult(ct_dx_s2, engine->encryptVector(dy2_o4)));

        orientationComputations += 4 * K;

        auto ct_s1 = engine->compareGtZeroPacked(ct_o1, K);
        auto ct_s2 = engine->compareGtZeroPacked(ct_o2, K);
        auto ct_s3 = engine->compareGtZeroPacked(ct_o3, K);
        auto ct_s4 = engine->compareGtZeroPacked(ct_o4, K);
        bootstrapCount += 4;
        signExtractions += 4;

        auto ct_xor12 = engine->eXor(ct_s1, ct_s2);
        auto ct_xor34 = engine->eXor(ct_s3, ct_s4);
        auto ct_inter = engine->eAnd(ct_xor12, ct_xor34);

        auto interResults = engine->decryptVector(ct_inter, K);
        for (int j = 0; j < K; j++)
        {
            results[group[j]] = interResults[j];
        }

        intersectionTests += K;
    };

    processGroup(groupZ, DROP_Z);
    processGroup(groupX, DROP_X);
    processGroup(groupY, DROP_Y);

    return results;
}

// ============================================================
// FULL BATCH WRAPPER (toutes les paires)
// ============================================================

std::vector<double> GeometryEngine::batchCheckIntersection3D_FULL(
    const std::vector<Segment> &segs1,
    const std::vector<Segment> &segs2)
{
    int N1 = segs1.size();
    int N2 = segs2.size();

    if (N1 == 0 || N2 == 0)
        return {};

    std::vector<double> results;
    results.reserve(N1 * N2);

    for (int i = 0; i < N1; i++)
    {
        const auto &s1 = segs1[i];
        auto row = batchCheckIntersection3D(s1, segs2);

        for (double v : row)
        {
            results.push_back(v);
        }
    }

    return results;
}

// ===== 3) Validation / stats =====

bool GeometryEngine::validatePoints(const IntPoint &p, const IntPoint &q, const IntPoint &r) const
{
    auto isValid = [](const IntPoint &pt)
    {
        return pt.x >= DroneConstants::MIN_COORDINATE &&
               pt.x <= DroneConstants::MAX_COORDINATE &&
               pt.y >= DroneConstants::MIN_COORDINATE &&
               pt.y <= DroneConstants::MAX_COORDINATE &&
               pt.z >= DroneConstants::MIN_COORDINATE &&
               pt.z <= DroneConstants::MAX_COORDINATE;
    };

    if (!isValid(p) || !isValid(q) || !isValid(r))
    {
        std::cerr << "Warning: Points exceed bounds" << std::endl;
    }

    return true;
}

void GeometryEngine::printStats() const
{
    std::cout << "\n=== Geometry Engine Statistics ===" << std::endl;
    std::cout << "Orientation computations: " << orientationComputations << std::endl;
    std::cout << "Sign extractions: " << signExtractions << std::endl;
    std::cout << "Intersection tests: " << intersectionTests << std::endl;
    double avg = intersectionTests ? (double)bootstrapCount / intersectionTests : 0.0;
    std::cout << "Bootstrap count: " << bootstrapCount
              << " (avg " << avg << " per test)" << std::endl;
}

// ============================================================
// BATCHING : Détection collision + évitement d'altitude
// ============================================================

CryptoEngine::CiphertextCKKS GeometryEngine::computeDistancesBatchTemporal(
    const Trajectory3D &trajA,
    const Trajectory3D &trajB)
{
    size_t N = trajA.size();
    std::vector<double> dx(N), dy(N), dz(N);
    for (size_t t = 0; t < N; t++)
    {
        dx[t] = trajA[t].x - trajB[t].x;
        dy[t] = trajA[t].y - trajB[t].y;
        dz[t] = trajA[t].z - trajB[t].z;
    }
    auto ct_dx = engine->encryptVector(dx);
    auto ct_dy = engine->encryptVector(dy);
    auto ct_dz = engine->encryptVector(dz);
    auto ct_dx2 = engine->mult(ct_dx, ct_dx);
    auto ct_dy2 = engine->mult(ct_dy, ct_dy);
    auto ct_dz2 = engine->mult(ct_dz, ct_dz);
    return engine->add(engine->add(ct_dx2, ct_dy2), ct_dz2);
}

CryptoEngine::CiphertextCKKS GeometryEngine::applyTemporalMask(
    const CiphertextCKKS &distances,
    size_t N,
    size_t horizonSteps)
{
    std::vector<double> mask(N, 0.0);
    for (size_t t = 0; t < horizonSteps && t < N; t++)
        mask[t] = 1.0;
    auto cc = engine->getCKKSContext();
    auto pt_mask = cc->MakeCKKSPackedPlaintext(mask);
    return cc->EvalMult(distances, pt_mask);
}

CryptoEngine::CiphertextCKKS GeometryEngine::detectCollisionInHorizon(
    const CiphertextCKKS &maskedDistances,
    double threshold)
{
    auto ct_thr = engine->constLike(maskedDistances, threshold * threshold);
    return engine->compareLE(maskedDistances, ct_thr);
}

CryptoEngine::CiphertextCKKS GeometryEngine::encodeCandidateAltitudes(
    double currentAltitude,
    double delta,
    int k)
{
    std::vector<double> candidates;
    for (int j = 1; j <= k / 2; j++)
    {
        candidates.push_back(currentAltitude + j * delta);
        candidates.push_back(currentAltitude - j * delta);
    }
    return engine->encryptVector(candidates);
}

CryptoEngine::CiphertextCKKS GeometryEngine::checkCandidatesAgainstDrone(
    const CiphertextCKKS &candidates,
    double droneAltitude,
    double threshold,
    int numCandidates)
{
    int totalSlots = engine->getSlotCount();

    std::vector<double> rep(totalSlots, droneAltitude);
    auto ct_drone = engine->encryptVector(rep);

    auto ct_diff = engine->sub(candidates, ct_drone);
    auto ct_diff2 = engine->mult(ct_diff, ct_diff);

    std::vector<double> thrVec(totalSlots, threshold * threshold);
    auto ct_thr = engine->encryptVector(thrVec);

    auto ct_diff_minus_thr = engine->sub(ct_diff2, ct_thr);

    int totalSlotsN = engine->getSlotCount();
    std::vector<double> normVec(totalSlotsN, 1.0 / 10000.0);
    auto cc = engine->getCKKSContext();
    auto pt_norm = cc->MakeCKKSPackedPlaintext(normVec);
    auto ct_normalized = cc->EvalMult(ct_diff_minus_thr, pt_norm);

    int padded = 1;
    while (padded < numCandidates)
        padded *= 2;

    return engine->compareGtZeroPacked(ct_normalized, padded);
}

double GeometryEngine::selectBestAltitude(
    const CiphertextCKKS &availability,
    double currentAltitude,
    double delta,
    int k)
{
    auto result = engine->decryptVector(availability);
    std::vector<double> candidates;
    for (int j = 1; j <= k / 2; j++)
    {
        candidates.push_back(currentAltitude + j * delta);
        candidates.push_back(currentAltitude - j * delta);
    }
    for (size_t j = 0; j < result.size() && j < candidates.size(); j++)
    {
        if (result[j] > 0.5)
            return candidates[j];
    }
    return currentAltitude;
}