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



// ===== 1) Fonctions en clair =====
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
        // segments parallèles -> on peut projeter sur XY par défaut
        return doSegmentsIntersectClear2D(s1, s2, GeometryEngine::DROP_Z);
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

    // cas général (2 boots)
    auto opp12 = engine->ltZero(engine->mult(o1, o2));
    auto opp34 = engine->ltZero(engine->mult(o3, o4));
    bootstrapCount += 2;
    auto generalBit = engine->eAnd(opp12, opp34); //=1 si intersection

    // Court-circuit 1 (option bench)-
    double gVal = engine->decryptValue(generalBit);
    if (gVal > 0.5) {
        intersectionTests++;
        return generalBit; // 2 boots seulement
    }

    //Détection des cas colinéaires / presque colinéaires: near-zero (4 boots)
    auto o1sq = engine->mult(o1,o1);
    auto o2sq = engine->mult(o2,o2);
    auto o3sq = engine->mult(o3,o3);
    auto o4sq = engine->mult(o4,o4);

    auto tau1 = engine->constLike(o1sq, tauOri*tauOri);
    auto tau2 = engine->constLike(o2sq, tauOri*tauOri);
    auto tau3 = engine->constLike(o3sq, tauOri*tauOri);
    auto tau4 = engine->constLike(o4sq, tauOri*tauOri);

    // z1 = 1 si o1² <= tau², sinon 0
    // compareGreaterThanZero(o1sq - tau1) vaut 1 si o1² > tau²
    // donc 1 - compare(...) vaut 1 si o1² <= tau²
    auto z1 = cc->EvalSub(engine->oneLike(o1sq),
              engine->compareGreaterThanZero(cc->EvalSub(o1sq, tau1))); 
    auto z2 = cc->EvalSub(engine->oneLike(o2sq),
              engine->compareGreaterThanZero(cc->EvalSub(o2sq, tau2)));
    auto z3 = cc->EvalSub(engine->oneLike(o3sq),
              engine->compareGreaterThanZero(cc->EvalSub(o3sq, tau3)));
    auto z4 = cc->EvalSub(engine->oneLike(o4sq),
              engine->compareGreaterThanZero(cc->EvalSub(o4sq, tau4)));
    bootstrapCount += 4;

    // Test si au moins une orientation est near-zero
    auto z_any = engine->eOr(engine->eOr(z1, z2), engine->eOr(z3, z4));

    // Court-circuit 2: si aucune orientation ~0 => parallèles
    double zanyVal = engine->decryptValue(z_any);
    if (zanyVal < 0.5) {
        intersectionTests++;
        return engine->constLike(z_any, 0.0); // 6 boots total
    }

    // cas colinéaires : tester si une extremité appartient à l'autre segment projeté
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
        if (p1.z != q1.z || p1.z != p2.z || p1.z != q2.z) {
           return engine->encryptValue(0.0); //pas de collisions si pas meme altitude
        }
        return checkSegmentIntersection2D(s1, s2, GeometryEngine::DROP_Z);
    }

    //cas non // 
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
    
    if (!isValid(p) || !isValid(q) || !isValid(r)) {
        std::cerr << "Warning: Points exceed bounds" << std::endl;
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


