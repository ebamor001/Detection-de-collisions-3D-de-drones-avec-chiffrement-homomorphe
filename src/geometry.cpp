#include "geometry.hpp"
#include <iostream>
#include <algorithm>

GeometryEngine::GeometryEngine(CryptoEngine* eng) : engine(eng) {
    if (!engine || !engine->isInitialized()) {
        throw std::runtime_error("GeometryEngine requires initialized CryptoEngine");
    }
}

// Fonctions en clair (inchangées)
GeometryEngine::Orientation GeometryEngine::orientationClear(
    const IntPoint& p, const IntPoint& q, const IntPoint& r) {
    
    long val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    
    if (val == 0) return COLLINEAR;
    return (val > 0) ? CLOCKWISE : COUNTERCLOCKWISE;
}

bool GeometryEngine::onSegmentClear(
    const IntPoint& p, const IntPoint& q, const IntPoint& r) {
    
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) {
        return true;
    }
    return false;
}

bool GeometryEngine::doSegmentsIntersectClear(const Segment& seg1, const Segment& seg2) {
    const IntPoint& p1 = seg1.first;
    const IntPoint& q1 = seg1.second;
    const IntPoint& p2 = seg2.first;
    const IntPoint& q2 = seg2.second;
    
    Orientation o1 = orientationClear(p1, q1, p2);
    Orientation o2 = orientationClear(p1, q1, q2);
    Orientation o3 = orientationClear(p2, q2, p1);
    Orientation o4 = orientationClear(p2, q2, q1);
    
    if (o1 != o2 && o3 != o4) {
        return true;
    }
    
    if (o1 == COLLINEAR && onSegmentClear(p1, p2, q1)) return true;
    if (o2 == COLLINEAR && onSegmentClear(p1, q2, q1)) return true;
    if (o3 == COLLINEAR && onSegmentClear(p2, p1, q2)) return true;
    if (o4 == COLLINEAR && onSegmentClear(p2, q1, q2)) return true;
    
    return false;
}

// Fonction chiffrée de base (scalaire)
CryptoEngine::CiphertextCKKS GeometryEngine::computeOrientationValue(
    const IntPoint& p, const IntPoint& q, const IntPoint& r) {
    
    double dy1 = static_cast<double>(q.y - p.y);
    double dx2 = static_cast<double>(r.x - q.x);
    double dx1 = static_cast<double>(q.x - p.x);
    double dy2 = static_cast<double>(r.y - q.y);
    
    auto ct_dy1 = engine->encryptValue(dy1);
    auto ct_dx2 = engine->encryptValue(dx2);
    auto ct_dx1 = engine->encryptValue(dx1);
    auto ct_dy2 = engine->encryptValue(dy2);
    
    auto ct_prod1 = engine->mult(ct_dy1, ct_dx2);
    auto ct_prod2 = engine->mult(ct_dx1, ct_dy2);
    auto ct_val = engine->sub(ct_prod1, ct_prod2);
    
    orientationComputations++;
    return ct_val;
}



// Produit scalaire 2D chiffré: (ax*bx + ay*by)
static CryptoEngine::CiphertextCKKS dot2(
    CryptoEngine* eng,
    const CryptoEngine::CiphertextCKKS& ax,
    const CryptoEngine::CiphertextCKKS& ay,
    const CryptoEngine::CiphertextCKKS& bx,
    const CryptoEngine::CiphertextCKKS& by)
{
    auto t1 = eng->mult(ax, bx);
    auto t2 = eng->mult(ay, by);
    return eng->add(t1, t2);
}

// on-segment 100% chiffré: z_orient * [ -eps <= s <= uu+eps ]
// où s = (q-p)·(r-p), uu = (r-p)·(r-p)
// on-segment 100% chiffré avec near-zero protocole normal
CryptoEngine::CiphertextCKKS GeometryEngine::onSegmentHE_full(
    const IntPoint& p, const IntPoint& q, const IntPoint& r,
    double tauOri, double epsProj)
{
    auto cc = engine->getCKKSContext();

    // Coordonnées chiffrées
    auto px = engine->encryptValue((double)p.x);
    auto py = engine->encryptValue((double)p.y);
    auto qx = engine->encryptValue((double)q.x);
    auto qy = engine->encryptValue((double)q.y);
    auto rx = engine->encryptValue((double)r.x);
    auto ry = engine->encryptValue((double)r.y);

    // Vecteurs u=r-p, v=q-p
    auto ux = engine->sub(rx, px);
    auto uy = engine->sub(ry, py);
    auto vx = engine->sub(qx, px);
    auto vy = engine->sub(qy, py);

    auto s  = dot2(engine, vx, vy, ux, uy); // v·u
    auto uu = dot2(engine, ux, uy, ux, uy); // u·u (>= 0)

    // Near-zero protocole normal: |o| <= tauOri (2 compares)
    auto o = computeOrientationValue(p, q, r);
    auto tauCt  = engine->constLike(o, tauOri);
    auto nego   = engine->negate(o);

    auto gt_pos = engine->compareGT(o, tauCt);    // 1 si o > +tau
    auto gt_neg = engine->compareGT(nego, tauCt); // 1 si -o > +tau (donc o < -tau)
    
    // near0 = NOT(gt_pos OR gt_neg)
    auto one = engine->oneLike(gt_pos);
    // OR = 1 - (1-gt_pos)*(1-gt_neg)
    auto outside = cc->EvalSub(one, cc->EvalMult(cc->EvalSub(one, gt_pos), 
                                                  cc->EvalSub(one, gt_neg)));
    auto near0 = cc->EvalSub(one, outside);  // NOT(OR)
    bootstrapCount += 2;

    // Fenêtre de projection: -epsProj <= s <= uu + epsProj (2 compares)
    auto eps = engine->constLike(s, epsProj);
    auto lo = engine->negate(eps);           // -eps
    auto hi = engine->add(uu, eps);          // uu + eps
    auto geLo = engine->compareGE(s, lo);    // s >= -eps
    auto leHi = engine->compareLE(s, hi);    // s <= uu+eps
    auto inProj = engine->mult(geLo, leHi);  // AND
    bootstrapCount += 2;

    // onSeg = near0 AND inProj
    return engine->eAnd(near0, inProj);
}

// Version avec court-circuit (corrigée)
CryptoEngine::CiphertextCKKS GeometryEngine::checkSegmentIntersectionBasic(
    const Segment& seg1, const Segment& seg2) {

    const double tauOri  = 1e-5;
    const double epsProj = 5e-6;
    auto cc = engine->getCKKSContext();

    const auto& p1 = seg1.first;  const auto& q1 = seg1.second;
    const auto& p2 = seg2.first;  const auto& q2 = seg2.second;

    // 1) Orientations (CKKS pur, 0 bootstrap)
    auto o1 = computeOrientationValue(p1, q1, p2);
    auto o2 = computeOrientationValue(p1, q1, q2);
    auto o3 = computeOrientationValue(p2, q2, p1);
    auto o4 = computeOrientationValue(p2, q2, q1);

    // 2) CAS GÉNÉRAL d'abord
    auto b1 = engine->compareGreaterThanZero(o1);
    auto b2 = engine->compareGreaterThanZero(o2);
    auto b3 = engine->compareGreaterThanZero(o3);
    auto b4 = engine->compareGreaterThanZero(o4);
    bootstrapCount += 4;

    auto xor12 = engine->eXor(b1, b2);
    auto xor34 = engine->eXor(b3, b4);
    auto generalBit = engine->eAnd(xor12, xor34);

    // COURT-CIRCUIT 1 (DEV/BENCH)
    double gVal = engine->decryptValue(generalBit);
    if (gVal > 0.5) {
        intersectionTests++;
        return generalBit;
    }

    // 3) Test colinéarité - CORRECTION: utiliser engine->
    auto z1 = engine->isNearZeroBand(o1, tauOri);
    auto z2 = engine->isNearZeroBand(o2, tauOri);
    auto z3 = engine->isNearZeroBand(o3, tauOri);
    auto z4 = engine->isNearZeroBand(o4, tauOri);
    bootstrapCount += 8;

    auto allCol = engine->eAnd(engine->eAnd(z1, z2), engine->eAnd(z3, z4));
    
    // COURT-CIRCUIT 2 (DEV/BENCH)
    double acVal = engine->decryptValue(allCol);
    if (acVal < 0.5) {
        intersectionTests++;
        return engine->constLike(allCol, 0.0);
    }

    // 4) Tests on-segment
    auto on1 = onSegmentHE_full(p1, p2, q1, tauOri, epsProj);
    auto on2 = onSegmentHE_full(p1, q2, q1, tauOri, epsProj);
    auto on3 = onSegmentHE_full(p2, p1, q2, tauOri, epsProj);
    auto on4 = onSegmentHE_full(p2, q1, q2, tauOri, epsProj);

    auto anyCol = engine->eOr(engine->eOr(on1, on2), engine->eOr(on3, on4));

    intersectionTests++;
    return anyCol;
}


// Version optimisée corrigée avec alignements et gestion des endpoints
CryptoEngine::CiphertextCKKS GeometryEngine::checkSegmentIntersectionOptimized(
    const Segment& seg1, const Segment& seg2) {

    const double tauOri  = 1e-5;
    const double epsProj = 5e-6;
    auto cc = engine->getCKKSContext();

    const auto& p1 = seg1.first;  const auto& q1 = seg1.second;
    const auto& p2 = seg2.first;  const auto& q2 = seg2.second;

    // A) Orientations (0 boot)
    auto o1 = computeOrientationValue(p1, q1, p2);
    auto o2 = computeOrientationValue(p1, q1, q2);
    auto o3 = computeOrientationValue(p2, q2, p1);
    auto o4 = computeOrientationValue(p2, q2, q1);

    // B) Cas général SANS normalisation (2 boots)
    auto prod12 = engine->mult(o1, o2);
    auto prod34 = engine->mult(o3, o4);
    auto opp12  = engine->ltZero(prod12);  // 1 si o1*o2 < 0
    auto opp34  = engine->ltZero(prod34);  // 1 si o3*o4 < 0
    bootstrapCount += 2;
    auto generalBit = engine->eAnd(opp12, opp34);

    // Court-circuit 1 (option bench)
    double gVal = engine->decryptValue(generalBit);
    if (gVal > 0.5) {
        intersectionTests++;
        return generalBit; // 2 boots seulement
    }

    // C) Near-zero pour O1..O4 (4 boots)
    auto o1sq = engine->mult(o1, o1);
    auto o2sq = engine->mult(o2, o2);
    auto o3sq = engine->mult(o3, o3);
    auto o4sq = engine->mult(o4, o4);

    // CORRECTION CRITIQUE: τ² aligné sur CHAQUE oi²
    auto tau1 = engine->constLike(o1sq, tauOri * tauOri);
    auto tau2 = engine->constLike(o2sq, tauOri * tauOri);
    auto tau3 = engine->constLike(o3sq, tauOri * tauOri);
    auto tau4 = engine->constLike(o4sq, tauOri * tauOri);

    // near0(oi) = NOT((oi² - τ²) > 0)
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

    // D) Tests on-segment pour endpoints/colinéaires (4 boots)
    auto on1 = onSegmentOptimized(p1, p2, q1, epsProj);
    auto on2 = onSegmentOptimized(p1, q2, q1, epsProj);
    auto on3 = onSegmentOptimized(p2, p1, q2, epsProj);
    auto on4 = onSegmentOptimized(p2, q1, q2, epsProj);

    // Combine avec les zi correspondants
    auto anyCol = engine->eOr(
        engine->eOr(engine->eAnd(z1, on1), engine->eAnd(z2, on2)),
        engine->eOr(engine->eAnd(z3, on3), engine->eAnd(z4, on4)));

    // Résultat final
    intersectionTests++;
    return engine->eOr(generalBit, anyCol); // 10 boots au pire
}

// On-segment optimisé reste inchangé
CryptoEngine::CiphertextCKKS GeometryEngine::onSegmentOptimized(
    const IntPoint& p, const IntPoint& q, const IntPoint& r, double eps) {

    // Version 100% chiffrée
    auto px = engine->encryptValue((double)p.x);
    auto py = engine->encryptValue((double)p.y);
    auto qx = engine->encryptValue((double)q.x);
    auto qy = engine->encryptValue((double)q.y);
    auto rx = engine->encryptValue((double)r.x);
    auto ry = engine->encryptValue((double)r.y);

    // u = r - p, v = q - p
    auto ux = engine->sub(rx, px);
    auto uy = engine->sub(ry, py);
    auto vx = engine->sub(qx, px);
    auto vy = engine->sub(qy, py);

    // s = v·u, uu = u·u
    auto s  = engine->add(engine->mult(vx, ux), engine->mult(vy, uy));
    auto uu = engine->add(engine->mult(ux, ux), engine->mult(uy, uy));

    // Test: s ∈ [-eps, uu+eps] <=> NOT((s+eps)(uu+eps-s) < 0)
    auto epsS = engine->constLike(s, eps);
    auto epsU = engine->constLike(uu, eps);

    auto term1 = engine->add(s, epsS);                  // s + eps
    auto term2 = engine->sub(engine->add(uu, epsU), s); // uu + eps - s
    auto prod = engine->mult(term1, term2);

    // NOT(prod < 0) = 1 - ltZero(prod)
    auto one = engine->oneLike(prod);
    auto onSeg = engine->sub(one, engine->ltZero(prod));
    
    bootstrapCount++;
    return onSeg;
}

// Point d'entrée principal
CryptoEngine::CiphertextCKKS GeometryEngine::checkSegmentIntersection(
    const Segment& seg1, const Segment& seg2) {
    return checkSegmentIntersectionBasic(seg1, seg2);
}

void GeometryEngine::printStats() const {
    std::cout << "\n=== Geometry Engine Statistics ===" << std::endl;
    std::cout << "Orientation computations: " << orientationComputations << std::endl;
    std::cout << "Sign extractions: " << signExtractions << std::endl;
    std::cout << "Intersection tests: " << intersectionTests << std::endl;
    std::cout << "Bootstrap count: " << bootstrapCount 
              << " (avg " << (double)bootstrapCount/intersectionTests 
              << " per test)" << std::endl;
}

bool GeometryEngine::validatePoints(const IntPoint& p, const IntPoint& q, const IntPoint& r) const {
    auto isValid = [](const IntPoint& pt) {
        return pt.x >= DroneConstants::MIN_COORDINATE && 
               pt.x <= DroneConstants::MAX_COORDINATE &&
               pt.y >= DroneConstants::MIN_COORDINATE && 
               pt.y <= DroneConstants::MAX_COORDINATE;
    };
    
    if (!isValid(p) || !isValid(q) || !isValid(r)) {
        std::cerr << "Warning: Points exceed bounds" << std::endl;
    }
    
    return true;
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

// Calcul complet orientation + signe (pour les tests)
CryptoEngine::CiphertextCKKS GeometryEngine::computeOrientation(
    const IntPoint& p, const IntPoint& q, const IntPoint& r) {
    
    auto ct_val = computeOrientationValue(p, q, r);
    
    if (engine->isSwitchingReady()) {
        return extractOrientationSign(ct_val);
    } else {
        return ct_val;
    }
}