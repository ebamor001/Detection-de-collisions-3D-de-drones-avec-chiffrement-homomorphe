#include "geometry.hpp"
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cmath>


GeometryEngine::GeometryEngine(CryptoEngine* eng) : engine(eng) {
    if (!engine || !engine->isInitialized()) {
        throw std::runtime_error("GeometryEngine requires initialized CryptoEngine");
    }
}


// helpers
static GeometryEngine::DropAxis chooseDropAxisFromNormal(long nx, long ny, long nz) {
    long ax = std::llabs(nx), ay = std::llabs(ny), az = std::llabs(nz);
    if (ax >= ay && ax >= az) return GeometryEngine::DROP_X;
    if (ay >= ax && ay >= az) return GeometryEngine::DROP_Y;
    return GeometryEngine::DROP_Z;
}

static std::pair<long,long> project2D(const IntPoint& p, GeometryEngine::DropAxis drop) {
    if (drop == GeometryEngine::DROP_X) return {p.y, p.z};
    if (drop == GeometryEngine::DROP_Y) return {p.x, p.z};
    return {p.x, p.y};
}


GeometryEngine::Orientation GeometryEngine::orientationClear2D(
    const IntPoint& p, const IntPoint& q, const IntPoint& r, DropAxis drop)
{
    auto [px,py] = project2D(p, drop);
    auto [qx,qy] = project2D(q, drop);
    auto [rx,ry] = project2D(r, drop);

    long val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy);

    if (val == 0) return COLLINEAR;
    return (val > 0) ? CLOCKWISE : COUNTERCLOCKWISE;
}

bool GeometryEngine::onSegmentClear2D(
    const IntPoint& p, const IntPoint& q, const IntPoint& r, DropAxis drop)
{
    auto [px,py] = project2D(p, drop);
    auto [qx,qy] = project2D(q, drop);
    auto [rx,ry] = project2D(r, drop);

    return (qx <= std::max(px, rx) && qx >= std::min(px, rx) &&
            qy <= std::max(py, ry) && qy >= std::min(py, ry));
}


bool GeometryEngine::doSegmentsIntersectClear2D(
    const Segment& s1, const Segment& s2, DropAxis drop)
{
    const auto& p1 = s1.first;  const auto& q1 = s1.second;
    const auto& p2 = s2.first;  const auto& q2 = s2.second;

    auto o1 = orientationClear2D(p1,q1,p2,drop);
    auto o2 = orientationClear2D(p1,q1,q2,drop);
    auto o3 = orientationClear2D(p2,q2,p1,drop);
    auto o4 = orientationClear2D(p2,q2,q1,drop);

    if (o1 != o2 && o3 != o4) return true;

    if (o1 == COLLINEAR && onSegmentClear2D(p1,p2,q1,drop)) return true;
    if (o2 == COLLINEAR && onSegmentClear2D(p1,q2,q1,drop)) return true;
    if (o3 == COLLINEAR && onSegmentClear2D(p2,p1,q2,drop)) return true;
    if (o4 == COLLINEAR && onSegmentClear2D(p2,q1,q2,drop)) return true;

    return false;
}

bool GeometryEngine::doSegmentsIntersectClear3D(const Segment& s1, const Segment& s2)
{
    const auto& p1 = s1.first;  const auto& q1 = s1.second;
    const auto& p2 = s2.first;  const auto& q2 = s2.second;

    // d1 = q1-p1, d2 = q2-p2
    long d1x = q1.x - p1.x, d1y = q1.y - p1.y, d1z = q1.z - p1.z;
    long d2x = q2.x - p2.x, d2y = q2.y - p2.y, d2z = q2.z - p2.z;

    // n = d1 x d2
    long nx = d1y*d2z - d1z*d2y;
    long ny = d1z*d2x - d1x*d2z;
    long nz = d1x*d2y - d1y*d2x;
    if (nx==0 && ny==0 && nz==0) {
        long wx0 = p2.x - p1.x;
        long wy0 = p2.y - p1.y;
        long wz0 = p2.z - p1.z;

        long cx = wy0*d1z - wz0*d1y;
        long cy = wz0*d1x - wx0*d1z;
        long cz = wx0*d1y - wy0*d1x;

        if (cx != 0 || cy != 0 || cz != 0)
            return false;

        DropAxis drop = chooseDropAxisFromNormal(d1x, d1y, d1z);
        return doSegmentsIntersectClear2D(s1, s2, drop);
    }

    // coplanarity: (p2-p1)·n == 0
    long wx = p2.x - p1.x, wy = p2.y - p1.y, wz = p2.z - p1.z;
    long cop = wx*nx + wy*ny + wz*nz;
    if (cop != 0) return false;

    DropAxis drop = chooseDropAxisFromNormal(nx,ny,nz);
    return doSegmentsIntersectClear2D(s1, s2, drop);
}



// ===== 2) HE primitives: orientation/intersection (chiffrés) =====
CryptoEngine::CiphertextCKKS GeometryEngine::computeOrientationValue2D(
    const IntPoint& p, const IntPoint& q, const IntPoint& r, DropAxis drop)
{
    auto [px,py] = project2D(p, drop);
    auto [qx,qy] = project2D(q, drop);
    auto [rx,ry] = project2D(r, drop);

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

// Calcul complet orientation + signe (pour les tests)

CryptoEngine::CiphertextCKKS GeometryEngine::computeOrientation2D(
    const IntPoint& p, const IntPoint& q, const IntPoint& r, DropAxis drop)
{
    auto ct_val = computeOrientationValue2D(p, q, r, drop);
    if (engine->isSwitchingReady()) return extractOrientationSign(ct_val);
    return ct_val;
}

// Extraction du signe d'une orientation (pour les tests)
CryptoEngine::CiphertextCKKS GeometryEngine::extractOrientationSign(
    const CiphertextCKKS& orientationVal) {
    
    if (!engine->isSwitchingReady()) {
        throw std::runtime_error("Scheme switching not ready");
    }
    
    auto ct_sign = engine->compareGreaterThanZero(orientationVal);
    signExtractions++;
    return ct_sign;
}



CryptoEngine::CiphertextCKKS GeometryEngine::onSegment2D(
    const IntPoint& p, const IntPoint& q, const IntPoint& r,
    DropAxis drop, double eps)
{
    auto [px,py] = project2D(p, drop);
    auto [qx,qy] = project2D(q, drop);
    auto [rx,ry] = project2D(r, drop);

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

    auto s  = engine->add(engine->mult(vx, ux), engine->mult(vy, uy));
    auto uu = engine->add(engine->mult(ux, ux), engine->mult(uy, uy));

    auto epsS = engine->constLike(s, eps);
    auto epsU = engine->constLike(uu, eps);

    auto term1 = engine->add(s, epsS);
    auto term2 = engine->sub(engine->add(uu, epsU), s);
    auto prod  = engine->mult(term1, term2);

    auto one   = engine->oneLike(prod);
    auto onSeg = engine->sub(one, engine->ltZero(prod));

    bootstrapCount++;
    return onSeg;
}

CryptoEngine::CiphertextCKKS GeometryEngine::checkSegmentIntersection2D(
    const Segment& seg1, const Segment& seg2, DropAxis drop)
{
    const double tauOri  = 1e-5;
    const double epsProj = 5e-6;
    auto cc = engine->getCKKSContext();

    const auto& p1 = seg1.first;  const auto& q1 = seg1.second;
    const auto& p2 = seg2.first;  const auto& q2 = seg2.second;

    // Orientations sur projection
    auto o1 = computeOrientationValue2D(p1, q1, p2, drop);
    auto o2 = computeOrientationValue2D(p1, q1, q2, drop);
    auto o3 = computeOrientationValue2D(p2, q2, p1, drop);
    auto o4 = computeOrientationValue2D(p2, q2, q1, drop);

    // Cas général : orientations opposées (2 SS)
    auto opp12 = engine->ltZero(engine->mult(o1, o2));
    auto opp34 = engine->ltZero(engine->mult(o3, o4));
    bootstrapCount += 2;
    auto generalBit = engine->eAnd(opp12, opp34);

    // Cas colinéaires : near-zero sur chaque orientation (4 SS)
    auto o1sq = engine->mult(o1, o1);
    auto o2sq = engine->mult(o2, o2);
    auto o3sq = engine->mult(o3, o3);
    auto o4sq = engine->mult(o4, o4);

    auto tau1 = engine->constLike(o1sq, tauOri * tauOri);
    auto tau2 = engine->constLike(o2sq, tauOri * tauOri);
    auto tau3 = engine->constLike(o3sq, tauOri * tauOri);
    auto tau4 = engine->constLike(o4sq, tauOri * tauOri);

    // z_i = 1 si o_i² <= tau²  (orientation quasi-nulle)
    auto z1 = cc->EvalSub(engine->oneLike(o1sq),
              engine->compareGreaterThanZero(cc->EvalSub(o1sq, tau1)));
    auto z2 = cc->EvalSub(engine->oneLike(o2sq),
              engine->compareGreaterThanZero(cc->EvalSub(o2sq, tau2)));
    auto z3 = cc->EvalSub(engine->oneLike(o3sq),
              engine->compareGreaterThanZero(cc->EvalSub(o3sq, tau3)));
    auto z4 = cc->EvalSub(engine->oneLike(o4sq),
              engine->compareGreaterThanZero(cc->EvalSub(o4sq, tau4)));
    bootstrapCount += 4;

    // Test d'appartenance au segment pour chaque cas colinéaire (4 SS)
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
    const Segment& s1, const Segment& s2)
{
    const auto& p1 = s1.first;  const auto& q1 = s1.second;
    const auto& p2 = s2.first;  const auto& q2 = s2.second;

    //calcul du produit vectoriel
    long d1x=q1.x-p1.x, d1y=q1.y-p1.y, d1z=q1.z-p1.z;
    long d2x=q2.x-p2.x, d2y=q2.y-p2.y, d2z=q2.z-p2.z;
    long nx = d1y*d2z - d1z*d2y;
    long ny = d1z*d2x - d1x*d2z;
    long nz = d1x*d2y - d1y*d2x;
    //si les 2 segments sont //
    if (nx==0 && ny==0 && nz==0) {
        // segments parallèles : vérifier colinéarité réelle

        long wx0 = p2.x - p1.x;
        long wy0 = p2.y - p1.y;
        long wz0 = p2.z - p1.z;

        long cx = wy0*d1z - wz0*d1y;
        long cy = wz0*d1x - wx0*d1z;
        long cz = wx0*d1y - wy0*d1x;

        // parallèles mais PAS sur la même droite
        if (cx != 0 || cy != 0 || cz != 0) {
            return engine->encryptValue(0.0);
        }

        // colinéaires → projection 2D
        DropAxis drop = chooseDropAxisFromNormal(d1x, d1y, d1z);
        return checkSegmentIntersection2D(s1, s2, drop);
    }

    //calcul du produit mixte
    DropAxis drop = chooseDropAxisFromNormal(nx, ny, nz);
    long wx=p2.x-p1.x, wy=p2.y-p1.y, wz=p2.z-p1.z;
    double cop = double(wx*nx + wy*ny + wz*nz);

    // near-zero coplanarity (HE) : si produit mixte=0 ==> coplanaires
    const double tauCop = 1e-5;
    auto ct_cop = engine->encryptValue(cop);
    auto copOK  = engine->isNearZeroBand(ct_cop, tauCop);
    bootstrapCount += 2; 

    auto inter2D = checkSegmentIntersection2D(s1, s2, drop);

    return engine->eAnd(copOK, inter2D);
}

std::vector<double> GeometryEngine::batchCheckIntersection3D(
    const Segment& mySeg,
    const std::vector<Segment>& neighbors)
{
    int N = (int)neighbors.size();
    if (N == 0) return {};

    const auto& p1 = mySeg.first;
    const auto& q1 = mySeg.second;

    long d1x = q1.x - p1.x;
    long d1y = q1.y - p1.y;
    long d1z = q1.z - p1.z;

    std::vector<double> results(N, 0.0);
    std::vector<DropAxis> drops(N, DROP_Z);
    std::vector<int> candidates;
    std::vector<int> collinear_group;

    for (int i = 0; i < N; i++) {
        const auto& p2 = neighbors[i].first;
        const auto& q2 = neighbors[i].second;

        long d2x = q2.x - p2.x;
        long d2y = q2.y - p2.y;
        long d2z = q2.z - p2.z;

        long nx = d1y*d2z - d1z*d2y;
        long ny = d1z*d2x - d1x*d2z;
        long nz = d1x*d2y - d1y*d2x;

        if (nx == 0 && ny == 0 && nz == 0) {
            long wx0 = p2.x - p1.x;
            long wy0 = p2.y - p1.y;
            long wz0 = p2.z - p1.z;

            long cx = wy0*d1z - wz0*d1y;
            long cy = wz0*d1x - wx0*d1z;
            long cz = wx0*d1y - wy0*d1x;

            if (cx == 0 && cy == 0 && cz == 0) {
                collinear_group.push_back(i);
                drops[i] = chooseDropAxisFromNormal(d1x, d1y, d1z);
            }
        } else {
            long wx = p2.x - p1.x;
            long wy = p2.y - p1.y;
            long wz = p2.z - p1.z;

            long cop = wx*nx + wy*ny + wz*nz;

            if (cop == 0) {
                candidates.push_back(i);
                drops[i] = chooseDropAxisFromNormal(nx, ny, nz);
            }
        }
    }

    if (candidates.empty() && collinear_group.empty())
        return results;

    std::vector<int> groupZ, groupX, groupY;

    for (int idx : candidates) {
        if (drops[idx] == DROP_Z) groupZ.push_back(idx);
        else if (drops[idx] == DROP_X) groupX.push_back(idx);
        else groupY.push_back(idx);
    }

    int maxCap = (int)engine->getCompareSlots();

    auto processGroup = [&](const std::vector<int>& group, DropAxis drop) {
        if (group.empty()) return;

        int K = (int)group.size();

        auto [p1a, p1b] = project2D(p1, drop);
        auto [q1a, q1b] = project2D(q1, drop);

        double dy1 = double(q1b - p1b);
        double dx1 = double(q1a - p1a);

        for (int start = 0; start < K; start += maxCap) {
            int end = std::min(K, start + maxCap);
            int Kc = end - start;

            std::vector<double> dy1_vec(Kc, dy1);
            std::vector<double> dx1_vec(Kc, dx1);

            std::vector<double> dx2_o1(Kc), dy2_o1(Kc);
            std::vector<double> dx2_o2(Kc), dy2_o2(Kc);
            std::vector<double> dy_seg2(Kc), dx_seg2(Kc);
            std::vector<double> dx2_o3(Kc), dy2_o3(Kc);
            std::vector<double> dx2_o4(Kc), dy2_o4(Kc);

            for (int j = 0; j < Kc; j++) {
                int idx = group[start + j];

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
                engine->mult(ct_dx1, engine->encryptVector(dy2_o1))
            );

            auto ct_o2 = engine->sub(
                engine->mult(ct_dy1, engine->encryptVector(dx2_o2)),
                engine->mult(ct_dx1, engine->encryptVector(dy2_o2))
            );

            auto ct_dy_s2 = engine->encryptVector(dy_seg2);
            auto ct_dx_s2 = engine->encryptVector(dx_seg2);

            auto ct_o3 = engine->sub(
                engine->mult(ct_dy_s2, engine->encryptVector(dx2_o3)),
                engine->mult(ct_dx_s2, engine->encryptVector(dy2_o3))
            );

            auto ct_o4 = engine->sub(
                engine->mult(ct_dy_s2, engine->encryptVector(dx2_o4)),
                engine->mult(ct_dx_s2, engine->encryptVector(dy2_o4))
            );

            orientationComputations += 4 * Kc;

            auto ct_s1 = engine->compareGtZeroPacked(ct_o1, Kc);
            auto ct_s2 = engine->compareGtZeroPacked(ct_o2, Kc);
            auto ct_s3 = engine->compareGtZeroPacked(ct_o3, Kc);
            auto ct_s4 = engine->compareGtZeroPacked(ct_o4, Kc);

            bootstrapCount += 4;
            signExtractions += 4;

            auto ct_xor12 = engine->eXor(ct_s1, ct_s2);
            auto ct_xor34 = engine->eXor(ct_s3, ct_s4);

            auto ct_inter = engine->eAnd(ct_xor12, ct_xor34);

            auto interResults = engine->decryptVector(ct_inter, Kc);

            for (int j = 0; j < Kc; j++)
                results[group[start + j]] = interResults[j];

            intersectionTests += Kc;
        }
    };

    processGroup(groupZ, DROP_Z);
    processGroup(groupX, DROP_X);
    processGroup(groupY, DROP_Y);

    if (!collinear_group.empty()) {
        int K = (int)collinear_group.size();

        long d1d1 = d1x*d1x + d1y*d1y + d1z*d1z;

        for (int start = 0; start < K; start += maxCap) {
            int end = std::min(K, start + maxCap);
            int Kc = end - start;

            std::vector<double> wp_vec(Kc);
            std::vector<double> wq_vec(Kc);
            std::vector<double> d1d1_vec(Kc, double(d1d1));

            for (int j = 0; j < Kc; j++) {
                int idx = collinear_group[start + j];

                const auto& p2 = neighbors[idx].first;
                const auto& q2 = neighbors[idx].second;

                wp_vec[j] = double(
                    (p2.x - p1.x)*d1x +
                    (p2.y - p1.y)*d1y +
                    (p2.z - p1.z)*d1z
                );

                wq_vec[j] = double(
                    (q2.x - p1.x)*d1x +
                    (q2.y - p1.y)*d1y +
                    (q2.z - p1.z)*d1z
                );
            }

            auto ct_wp = engine->encryptVector(wp_vec);
            auto ct_wq = engine->encryptVector(wq_vec);
            auto ct_d1d1 = engine->encryptVector(d1d1_vec);

            auto lt_wp = engine->compareGtZeroPacked(engine->negate(ct_wp), Kc);
            auto lt_wq = engine->compareGtZeroPacked(engine->negate(ct_wq), Kc);

            auto gt_wp = engine->compareGtZeroPacked(engine->sub(ct_wp, ct_d1d1), Kc);
            auto gt_wq = engine->compareGtZeroPacked(engine->sub(ct_wq, ct_d1d1), Kc);

            bootstrapCount += 4;
            signExtractions += 4;

            auto both_below = engine->eAnd(lt_wp, lt_wq);
            auto both_above = engine->eAnd(gt_wp, gt_wq);

            auto not_below = engine->eNot(both_below);
            auto not_above = engine->eNot(both_above);

            auto ct_overlap = engine->eAnd(not_below, not_above);

            auto overlapResults = engine->decryptVector(ct_overlap, Kc);

            for (int j = 0; j < Kc; j++)
                results[collinear_group[start + j]] = overlapResults[j];

            intersectionTests += Kc;
        }
    }

    return results;
}




GeometryEngine::EncDropChoice GeometryEngine::chooseDropAxisEncrypted(
    const CiphertextCKKS& nx,
    const CiphertextCKKS& ny,
    const CiphertextCKKS& nz)
{
    auto cc = engine->getCKKSContext();

    // Compare nx², ny², nz² instead of |nx|, |ny|, |nz|
    auto nx2 = cc->EvalMult(nx, nx);
    auto ny2 = cc->EvalMult(ny, ny);
    auto nz2 = cc->EvalMult(nz, nz);

    // Normalize before scheme switching comparisons
    auto scale = engine->constLike(nx2, 1.0 / 1000000.0);
    nx2 = cc->EvalMult(nx2, scale);
    ny2 = cc->EvalMult(ny2, scale);
    nz2 = cc->EvalMult(nz2, scale);

    // bxy = 1 if nx² > ny²
    auto bxy = engine->compareGreaterThanZero(cc->EvalSub(nx2, ny2));

    auto one = engine->oneLike(bxy);
    auto not_bxy = cc->EvalSub(one, bxy);

    // maxXY = bxy * nx² + (1-bxy) * ny²
    auto maxXY = cc->EvalAdd(
        cc->EvalMult(bxy, nx2),
        cc->EvalMult(not_bxy, ny2)
    );

    // bxyz = 1 if max(nx², ny²) > nz²
    auto bxyz = engine->compareGreaterThanZero(cc->EvalSub(maxXY, nz2));

    auto not_bxyz = cc->EvalSub(engine->oneLike(bxyz), bxyz);

    EncDropChoice choice;

    // If bxyz = 1, max is X or Y.
    // If bxyz = 0, max is Z.
    choice.chooseX = cc->EvalMult(bxyz, bxy);
    choice.chooseY = cc->EvalMult(bxyz, not_bxy);
    choice.chooseZ = not_bxyz;

    bootstrapCount += 2;

    return choice;
}


CryptoEngine::CiphertextCKKS GeometryEngine::checkSegmentIntersection3DEncrypted(
    const CiphertextCKKS& p1x, const CiphertextCKKS& p1y, const CiphertextCKKS& p1z,
    const CiphertextCKKS& q1x, const CiphertextCKKS& q1y, const CiphertextCKKS& q1z,
    const CiphertextCKKS& p2x, const CiphertextCKKS& p2y, const CiphertextCKKS& p2z,
    const CiphertextCKKS& q2x, const CiphertextCKKS& q2y, const CiphertextCKKS& q2z
) {
    auto cc = engine->getCKKSContext();

    auto d1x = cc->EvalSub(q1x, p1x);
    auto d1y = cc->EvalSub(q1y, p1y);
    auto d1z = cc->EvalSub(q1z, p1z);

    auto d2x = cc->EvalSub(q2x, p2x);
    auto d2y = cc->EvalSub(q2y, p2y);
    auto d2z = cc->EvalSub(q2z, p2z);

    auto nx = cc->EvalSub(cc->EvalMult(d1y, d2z), cc->EvalMult(d1z, d2y));
    auto ny = cc->EvalSub(cc->EvalMult(d1z, d2x), cc->EvalMult(d1x, d2z));
    auto nz = cc->EvalSub(cc->EvalMult(d1x, d2y), cc->EvalMult(d1y, d2x));

    auto dropChoice = chooseDropAxisEncrypted(nx, ny, nz);

    auto selectA = [&](const CiphertextCKKS& x,
                       const CiphertextCKKS& y,
                       const CiphertextCKKS& z) {
        return cc->EvalAdd(
            cc->EvalAdd(
                cc->EvalMult(dropChoice.chooseX, y),
                cc->EvalMult(dropChoice.chooseY, x)
            ),
            cc->EvalMult(dropChoice.chooseZ, x)
        );
    };

    auto selectB = [&](const CiphertextCKKS& x,
                       const CiphertextCKKS& y,
                       const CiphertextCKKS& z) {
        return cc->EvalAdd(
            cc->EvalAdd(
                cc->EvalMult(dropChoice.chooseX, z),
                cc->EvalMult(dropChoice.chooseY, z)
            ),
            cc->EvalMult(dropChoice.chooseZ, y)
        );
    };

    auto p1a = selectA(p1x, p1y, p1z);
    auto p1b = selectB(p1x, p1y, p1z);
    auto q1a = selectA(q1x, q1y, q1z);
    auto q1b = selectB(q1x, q1y, q1z);

    auto p2a = selectA(p2x, p2y, p2z);
    auto p2b = selectB(p2x, p2y, p2z);
    auto q2a = selectA(q2x, q2y, q2z);
    auto q2b = selectB(q2x, q2y, q2z);

    auto wx = cc->EvalSub(p2x, p1x);
    auto wy = cc->EvalSub(p2y, p1y);
    auto wz = cc->EvalSub(p2z, p1z);

    auto cop = cc->EvalAdd(
        cc->EvalAdd(
            cc->EvalMult(wx, nx),
            cc->EvalMult(wy, ny)
        ),
        cc->EvalMult(wz, nz)
    );

    auto copOK = engine->isNearZeroBand(cop, 1.0);
    bootstrapCount += 2;

    auto d1a = cc->EvalSub(q1a, p1a);
    auto d1b = cc->EvalSub(q1b, p1b);

    auto d2a = cc->EvalSub(q2a, p2a);
    auto d2b = cc->EvalSub(q2b, p2b);

    auto o1 = cc->EvalSub(
        cc->EvalMult(d1b, cc->EvalSub(p2a, q1a)),
        cc->EvalMult(d1a, cc->EvalSub(p2b, q1b))
    );

    auto o2 = cc->EvalSub(
        cc->EvalMult(d1b, cc->EvalSub(q2a, q1a)),
        cc->EvalMult(d1a, cc->EvalSub(q2b, q1b))
    );

    auto o3 = cc->EvalSub(
        cc->EvalMult(d2b, cc->EvalSub(p1a, q2a)),
        cc->EvalMult(d2a, cc->EvalSub(p1b, q2b))
    );

    auto o4 = cc->EvalSub(
        cc->EvalMult(d2b, cc->EvalSub(q1a, q2a)),
        cc->EvalMult(d2a, cc->EvalSub(q1b, q2b))
    );

    orientationComputations += 4;

    auto norm = engine->constLike(o1, 1.0 / 100000.0);

    auto p12 = cc->EvalMult(cc->EvalMult(o1, o2), norm);
    auto p34 = cc->EvalMult(cc->EvalMult(o3, o4), norm);

    auto opp12 = engine->ltZero(p12);
    auto opp34 = engine->ltZero(p34);
    bootstrapCount += 2;

    auto generalInter = engine->eAnd(opp12, opp34);

    auto nearZero = [&](const CiphertextCKKS& v) {
        return engine->isNearZeroBand(v, 1.0);
    };

    auto z1 = nearZero(o1);
    auto z2 = nearZero(o2);
    auto z3 = nearZero(o3);
    auto z4 = nearZero(o4);
    bootstrapCount += 8;

    auto onSegmentEnc = [&](const CiphertextCKKS& pa,
                            const CiphertextCKKS& pb,
                            const CiphertextCKKS& qa,
                            const CiphertextCKKS& qb,
                            const CiphertextCKKS& ra,
                            const CiphertextCKKS& rb) {
        auto ua = cc->EvalSub(ra, pa);
        auto ub = cc->EvalSub(rb, pb);

        auto va = cc->EvalSub(qa, pa);
        auto vb = cc->EvalSub(qb, pb);

        auto s = cc->EvalAdd(
            cc->EvalMult(va, ua),
            cc->EvalMult(vb, ub)
        );

        auto uu = cc->EvalAdd(
            cc->EvalMult(ua, ua),
            cc->EvalMult(ub, ub)
        );

        auto epsS = engine->constLike(s, 1e-5);
        auto epsU = engine->constLike(uu, 1e-5);

        auto term1 = cc->EvalAdd(s, epsS);
        auto term2 = cc->EvalSub(cc->EvalAdd(uu, epsU), s);
        auto prod = cc->EvalMult(term1, term2);

        auto outside = engine->ltZero(prod);
        auto one = engine->oneLike(outside);

        return cc->EvalSub(one, outside);
    };

    auto on1 = onSegmentEnc(p1a, p1b, p2a, p2b, q1a, q1b);
    auto on2 = onSegmentEnc(p1a, p1b, q2a, q2b, q1a, q1b);
    auto on3 = onSegmentEnc(p2a, p2b, p1a, p1b, q2a, q2b);
    auto on4 = onSegmentEnc(p2a, p2b, q1a, q1b, q2a, q2b);
    bootstrapCount += 4;

    auto collinearInter = engine->eOr(
        engine->eOr(engine->eAnd(z1, on1), engine->eAnd(z2, on2)),
        engine->eOr(engine->eAnd(z3, on3), engine->eAnd(z4, on4))
    );

    auto inter2D = engine->eOr(generalInter, collinearInter);

    intersectionTests++;

    return engine->eAnd(copOK, inter2D);
}

// ===== 3) Validation / stats =====

bool GeometryEngine::validatePoints(const IntPoint& p, const IntPoint& q, const IntPoint& r) const {
    auto isValid = [](const IntPoint& pt) {
        return pt.x >= DroneConstants::MIN_COORDINATE && 
               pt.x <= DroneConstants::MAX_COORDINATE &&
               pt.y >= DroneConstants::MIN_COORDINATE && 
               pt.y <= DroneConstants::MAX_COORDINATE &&
               pt.z >= DroneConstants::MIN_COORDINATE && 
               pt.z <= DroneConstants::MAX_COORDINATE ;
    };
    
    if (!isValid(p)) {
        std::cerr << "Error: Point p " << p << " exceeds bounds ["
                  << DroneConstants::MIN_COORDINATE << ", "
                  << DroneConstants::MAX_COORDINATE << "]" << std::endl;
        return false;
    }
    if (!isValid(q)) {
        std::cerr << "Error: Point q " << q << " exceeds bounds ["
                  << DroneConstants::MIN_COORDINATE << ", "
                  << DroneConstants::MAX_COORDINATE << "]" << std::endl;
        return false;
    }
    if (!isValid(r)) {
        std::cerr << "Error: Point r " << r << " exceeds bounds ["
                  << DroneConstants::MIN_COORDINATE << ", "
                  << DroneConstants::MAX_COORDINATE << "]" << std::endl;
        return false;
    }
    return true;
}

void GeometryEngine::printStats() const {
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
    const Trajectory3D& trajA,
    const Trajectory3D& trajB)
{
    size_t N = trajA.size();
    std::vector<double> dx(N), dy(N), dz(N);
    for (size_t t = 0; t < N; t++) {
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
    const CiphertextCKKS& distances,
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
    const CiphertextCKKS& maskedDistances,
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
    for (int j = 1; j <= k/2; j++) {
        candidates.push_back(currentAltitude + j * delta);
        candidates.push_back(currentAltitude - j * delta);
    }
    return engine->encryptVector(candidates);
}

CryptoEngine::CiphertextCKKS GeometryEngine::checkCandidatesAgainstDrone(
    const CiphertextCKKS& candidates,
    double droneAltitude,
    double threshold,
    int numCandidates)
{
    int totalSlots = engine->getSlotCount();

    std::vector<double> rep(totalSlots, droneAltitude);
    auto ct_drone = engine->encryptVector(rep);

    auto ct_diff  = engine->sub(candidates, ct_drone);
    auto ct_diff2 = engine->mult(ct_diff, ct_diff);

    std::vector<double> thrVec(totalSlots, threshold * threshold);
    auto ct_thr = engine->encryptVector(thrVec);

    auto ct_diff_minus_thr = engine->sub(ct_diff2, ct_thr);

    // Normaliser pour ramener dans [-1, 1] avant compareGtZeroPacked
    // On divise par une valeur > max possible de diff_minus_thr
    // max diff = (180-120)² - 15² = 3600 - 225 = 3375, on prend 10000
    int totalSlotsN = engine->getSlotCount();
    std::vector<double> normVec(totalSlotsN, 1.0 / 10000.0);
    auto cc = engine->getCKKSContext();
    auto pt_norm = cc->MakeCKKSPackedPlaintext(normVec);
    auto ct_normalized = cc->EvalMult(ct_diff_minus_thr, pt_norm);

    int padded = 1;
    while (padded < numCandidates) padded *= 2;

    return engine->compareGtZeroPacked(ct_normalized, padded);
}

double GeometryEngine::selectBestAltitude(
    const CiphertextCKKS& availability,
    double currentAltitude,
    double delta,
    int k)
{
    auto result = engine->decryptVector(availability,k);
    std::vector<double> candidates;
    for (int j = 1; j <= k/2; j++) {
        candidates.push_back(currentAltitude + j * delta);
        candidates.push_back(currentAltitude - j * delta);
    }
    for (size_t j = 0; j < result.size() && j < candidates.size(); j++) {
        if (result[j] > 0.5)
            return candidates[j];
    }
    return currentAltitude;
}

