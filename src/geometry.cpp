#include "geometry.hpp"
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <chrono>

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
        // segments paralleles : collision seulement s'ils sont colineaires (meme droite 3D)
        // verifier w x d1 == 0  (w = p2-p1)
        long wx0 = p2.x-p1.x, wy0 = p2.y-p1.y, wz0 = p2.z-p1.z;
        long cx = wy0*d1z - wz0*d1y;
        long cy = wz0*d1x - wx0*d1z;
        long cz = wx0*d1y - wy0*d1x;
        if (cx != 0 || cy != 0 || cz != 0) return false; // paralleles non colineaires
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

// Version batchee de checkSegmentIntersection3D.
// Au lieu de tester 1 paire a la fois (N appels = N*8 scheme switches),
// on teste N paires en parallele (1 appel = 6 scheme switches quel que soit N).
//
// Fonctionnement :
//   1. Cross product en clair pour chaque paire (gratuit, pas de chiffrement)
//   2. Packer les N produits mixtes dans 1 ciphertext → test coplanarite batche (2 SS)
//   3. Grouper les paires par axe de projection (drop X, Y ou Z)
//   4. Pour chaque groupe, packer les 4 orientations → 4 scheme switches pour le groupe
//   5. Logique booleenne XOR + AND (CKKS pur, 0 SS)
//   6. Combiner avec le resultat de coplanarite
//
// Resultat : un vecteur de doubles, 1 valeur par voisin.
//   > 0.5 = collision, < 0.5 = pas de collision.

std::vector<double> GeometryEngine::batchCheckIntersection3D(
    const Segment& mySeg,
    const std::vector<Segment>& neighbors)
{
    int N = (int)neighbors.size();
    if (N == 0) return {};

    const auto& p1 = mySeg.first;
    const auto& q1 = mySeg.second;

    // direction de mon segment (en clair, c'est MON segment)
    long d1x = q1.x - p1.x, d1y = q1.y - p1.y, d1z = q1.z - p1.z;

    // etape 1 : cross product et produit mixte en clair pour chaque paire
    // c'est gratuit car on calcule sur des long, pas sur des ciphertexts
    std::vector<double> cops(N);
    std::vector<DropAxis> drops(N);
    std::vector<bool> isParallel(N, false);
    std::vector<double> results(N, 0.0);
    // indices des paires colineaires → FHE overlap (pas FHE orientation qui vaut 0)
    std::vector<int> collinear_group;

    for (int i = 0; i < N; i++) {
        const auto& p2 = neighbors[i].first;
        const auto& q2 = neighbors[i].second;

        // direction du segment voisin
        long d2x = q2.x - p2.x, d2y = q2.y - p2.y, d2z = q2.z - p2.z;

        // produit vectoriel n = d1 x d2
        long nx = d1y*d2z - d1z*d2y;
        long ny = d1z*d2x - d1x*d2z;
        long nz = d1x*d2y - d1y*d2x;

        if (nx == 0 && ny == 0 && nz == 0) {
            // segments paralleles : colineaires seulement si w x d1 == 0
            long wx0 = p2.x-p1.x, wy0 = p2.y-p1.y, wz0 = p2.z-p1.z;
            long cx = wy0*d1z - wz0*d1y;
            long cy = wz0*d1x - wx0*d1z;
            long cz = wx0*d1y - wy0*d1x;
            isParallel[i] = true;
            if (cx != 0 || cy != 0 || cz != 0) {
                cops[i] = 9999.0;  // paralleles non colineaires → pas de collision
                drops[i] = DROP_Z;
            } else {
                // Colineaires : orientations 2D = 0 → FHE orientation inutilisable.
                // On utilise un test d'overlap FHE different (4 comparaisons TFHE batch).
                // Le groupe sera traite par processCollinearGroup apres la boucle.
                collinear_group.push_back(i);
                cops[i]  = 9999.0;  // exclure du batch FHE orientation
                drops[i] = DROP_Z;
            }
        } else {
            // produit mixte : (p2-p1) . n
            long wx = p2.x - p1.x, wy = p2.y - p1.y, wz = p2.z - p1.z;
            cops[i] = double(wx*nx + wy*ny + wz*nz);
            drops[i] = chooseDropAxisFromNormal(nx, ny, nz);
        }
    }

    // pré-filtre clair : pour coordonnees entieres, cops[i]==0 est exact
    // si cops[i] != 0 → definitiquement non coplanaire, zero FHE requis
    std::vector<int> candidates;
    for (int i = 0; i < N; i++)
        if (cops[i] == 0.0) candidates.push_back(i);

    if (candidates.empty() && collinear_group.empty()) return results;  // zero FHE

    // etape 2 : coplanarite FHE — decoupee en chunks de switchValues slots
    // isNearZeroBand appelle compareGreaterThanZero qui est limite a switchValues slots.
    // Si M > switchValues, on traite en plusieurs appels FHE.
    int M        = (int)candidates.size();
    int maxSlotC = (int)engine->getCompareSlots();  // = config.switchValues
    std::vector<double> copResults(N, 0.0);

    for (int c0 = 0; c0 < M; c0 += maxSlotC) {
        int c1 = std::min(M, c0 + maxSlotC);
        int Mc = c1 - c0;
        std::vector<double> cops_chunk(Mc);
        for (int j = 0; j < Mc; j++) cops_chunk[j] = cops[candidates[c0 + j]];
        auto ct_cops  = engine->encryptVector(cops_chunk);
        auto ct_copOK = engine->isNearZeroBand(ct_cops, 1e-5);
        bootstrapCount += 2;
        auto copRes = engine->decryptVector(ct_copOK, Mc);
        for (int j = 0; j < Mc; j++) copResults[candidates[c0 + j]] = copRes[j];
    }

    // etape 3 : grouper par axe de projection (candidats coplanaires seulement)
    std::vector<int> groupZ, groupX, groupY;
    for (int i : candidates) {
        if (copResults[i] < 0.5) continue;
        if (drops[i] == DROP_Z) groupZ.push_back(i);
        else if (drops[i] == DROP_X) groupX.push_back(i);
        else groupY.push_back(i);
    }

    // etape 4 : pour chaque groupe, batch des 4 orientations 2D
    // cette lambda traite un groupe de paires qui ont le meme axe de projection
    auto processGroup = [&](const std::vector<int>& group, DropAxis drop) {
        if (group.empty()) return;
        int K      = (int)group.size();
        int maxCap = (int)engine->getCompareSlots(); // = config.switchValues

        auto [p1a, p1b] = project2D(p1, drop);
        auto [q1a, q1b] = project2D(q1, drop);
        double dy1 = double(q1b - p1b);
        double dx1 = double(q1a - p1a);

        // Découper en sous-batches de taille <= maxCap pour respecter
        // la contrainte EvalCompareSchemeSwitching(numSlots == switchValues).
        // Chaque chunk utilise 4 SS ; le gain SIMD reste total pour K <= maxCap.
        for (int start = 0; start < K; start += maxCap) {
            int end  = std::min(K, start + maxCap);
            int Kc   = end - start;           // taille du chunk courant

            std::vector<double> dy1_vec(Kc, dy1), dx1_vec(Kc, dx1);
            std::vector<double> dx2_o1(Kc), dy2_o1(Kc);
            std::vector<double> dx2_o2(Kc), dy2_o2(Kc);
            std::vector<double> dy_seg2(Kc), dx_seg2(Kc);
            std::vector<double> dx2_o3(Kc), dy2_o3(Kc);
            std::vector<double> dx2_o4(Kc), dy2_o4(Kc);

            for (int j = 0; j < Kc; j++) {
                int idx = group[start + j];
                auto [p2a, p2b] = project2D(neighbors[idx].first,  drop);
                auto [q2a, q2b] = project2D(neighbors[idx].second, drop);

                dx2_o1[j] = double(p2a - q1a); dy2_o1[j] = double(p2b - q1b);
                dx2_o2[j] = double(q2a - q1a); dy2_o2[j] = double(q2b - q1b);
                dy_seg2[j] = double(q2b - p2b); dx_seg2[j] = double(q2a - p2a);
                dx2_o3[j] = double(p1a - q2a); dy2_o3[j] = double(p1b - q2b);
                dx2_o4[j] = double(q1a - q2a); dy2_o4[j] = double(q1b - q2b);
            }

            auto ct_dy1   = engine->encryptVector(dy1_vec);
            auto ct_dx1   = engine->encryptVector(dx1_vec);
            auto ct_dy_s2 = engine->encryptVector(dy_seg2);
            auto ct_dx_s2 = engine->encryptVector(dx_seg2);

            auto ct_o1 = engine->sub(engine->mult(ct_dy1,   engine->encryptVector(dx2_o1)),
                                     engine->mult(ct_dx1,   engine->encryptVector(dy2_o1)));
            auto ct_o2 = engine->sub(engine->mult(ct_dy1,   engine->encryptVector(dx2_o2)),
                                     engine->mult(ct_dx1,   engine->encryptVector(dy2_o2)));
            auto ct_o3 = engine->sub(engine->mult(ct_dy_s2, engine->encryptVector(dx2_o3)),
                                     engine->mult(ct_dx_s2, engine->encryptVector(dy2_o3)));
            auto ct_o4 = engine->sub(engine->mult(ct_dy_s2, engine->encryptVector(dx2_o4)),
                                     engine->mult(ct_dx_s2, engine->encryptVector(dy2_o4)));

            orientationComputations += 4 * Kc;

            auto t0_ss = std::chrono::high_resolution_clock::now();
            auto ct_s1 = engine->compareGtZeroPacked(ct_o1, Kc);
            auto t1_ss = std::chrono::high_resolution_clock::now();
            double ss_ms_reel = std::chrono::duration<double,std::milli>(t1_ss-t0_ss).count();
            std::cout << "[BENCH] 1 scheme switch = " << ss_ms_reel << " ms\n";
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
        } // fin chunk
    };

    // traiter chaque groupe orientation (non-colineaires coplanaires)
    processGroup(groupZ, DROP_Z);
    processGroup(groupX, DROP_X);
    processGroup(groupY, DROP_Y);

    // ── Traiter les segments colineaires avec FHE overlap ──────────────────────
    // Les orientations 2D sont toutes = 0 pour des segments colineaires.
    // On utilise un test different : overlap sur la droite 3D.
    // overlap = NOT(les deux extremites de seg2 sont en dehors du meme cote de seg1)
    //         = NOT(wp2<0 ET wq2<0) ET NOT(wp2>d1d1 ET wq2>d1d1)
    // 4 scheme-switches (comme le test d'orientation standard).
    if (!collinear_group.empty()) {
        int Kc = (int)collinear_group.size();
        long d1d1 = d1x*d1x + d1y*d1y + d1z*d1z;

        std::vector<double> wp_vec(Kc), wq_vec(Kc), d1d1_vec(Kc, double(d1d1));
        for (int j = 0; j < Kc; j++) {
            int idx = collinear_group[j];
            const auto& p2c = neighbors[idx].first;
            const auto& q2c = neighbors[idx].second;
            wp_vec[j] = double((p2c.x-p1.x)*d1x + (p2c.y-p1.y)*d1y + (p2c.z-p1.z)*d1z);
            wq_vec[j] = double((q2c.x-p1.x)*d1x + (q2c.y-p1.y)*d1y + (q2c.z-p1.z)*d1z);
        }

        auto ct_wp   = engine->encryptVector(wp_vec);
        auto ct_wq   = engine->encryptVector(wq_vec);
        auto ct_d1d1 = engine->encryptVector(d1d1_vec);

        // SS1 : lt_wp = 1 ou wp < 0  (compareGtZeroPacked(-wp))
        auto ct_s1 = engine->compareGtZeroPacked(engine->negate(ct_wp), Kc);
        // SS2 : lt_wq = 1 ou wq < 0
        auto ct_s2 = engine->compareGtZeroPacked(engine->negate(ct_wq), Kc);
        // SS3 : gt_wp = 1 ou wp > d1d1  (compareGtZeroPacked(wp - d1d1))
        auto ct_s3 = engine->compareGtZeroPacked(engine->sub(ct_wp, ct_d1d1), Kc);
        // SS4 : gt_wq = 1 ou wq > d1d1
        auto ct_s4 = engine->compareGtZeroPacked(engine->sub(ct_wq, ct_d1d1), Kc);
        bootstrapCount += 4;
        signExtractions += 4;

        // both_below = lt_wp AND lt_wq  →  not_both_below = eNot(eAnd(...))
        auto not_both_below = engine->eNot(engine->eAnd(ct_s1, ct_s2));
        // both_above = gt_wp AND gt_wq  →  not_both_above = eNot(eAnd(...))
        auto not_both_above = engine->eNot(engine->eAnd(ct_s3, ct_s4));
        // overlap = not_both_below AND not_both_above
        auto ct_overlap = engine->eAnd(not_both_below, not_both_above);

        auto overlapRes = engine->decryptVector(ct_overlap, Kc);
        for (int j = 0; j < Kc; j++)
            results[collinear_group[j]] = overlapRes[j];

        intersectionTests += Kc;
    }

    return results;
}


// ===== 3) Validation / stats =====

// ── Version chiffree de batchCheckIntersection3D ─────────────────────────────
// Prend deux ciphertexts [x1,y1,z1,x2,y2,z2] — un par drone
// Fait tout le calcul sous FHE : produit vectoriel, produit mixte, orientations
// Renvoie 1.0 si collision, 0.0 sinon
double GeometryEngine::batchCheckIntersection3D_encrypted(
    const CiphertextCKKS& ct_seg1,
    const CiphertextCKKS& ct_seg2)
{
    auto cc = engine->getCKKSContext();

    // ── Extraire les coordonnees des deux segments depuis les ciphertexts ──────
    // ct_seg1 = [x1, y1, z1, x2, y2, z2] pour le segment d'Alice
    // ct_seg2 = [x1, y1, z1, x2, y2, z2] pour le segment de Bob
    // On extrait chaque coordonnee en dechiffrant (Alice a deja chiffre avec pk)
    // Bob peut faire ce calcul car il a le contexte et les eval keys

    // Vecteur de selection pour extraire chaque slot
    // slot i = 1.0 dans le vecteur de selection, 0 ailleurs
    auto extractSlot = [&](const CiphertextCKKS& ct, int slot) -> CiphertextCKKS {
        std::vector<double> mask(6, 0.0);
        mask[slot] = 1.0;
        auto pt_mask = cc->MakeCKKSPackedPlaintext(mask);
        auto ct_masked = cc->EvalMult(ct, pt_mask);
        // Somme de rotation pour ramener la valeur en slot 0
        auto ct_sum = ct_masked;
        for (int i = 1; i < 6; i *= 2) {
            auto ct_rot = cc->EvalRotate(ct_sum, slot);
            ct_sum = cc->EvalAdd(ct_sum, ct_rot);
        }
        // Garder seulement slot 0
        std::vector<double> keep(6, 0.0);
        keep[0] = 1.0;
        auto pt_keep = cc->MakeCKKSPackedPlaintext(keep);
        return cc->EvalMult(ct_sum, pt_keep);
    };

    // Extraire les 6 coordonnees de chaque segment
    // Segment 1 (Alice) : P1=(x1,y1,z1), Q1=(x2,y2,z2)
    auto ct_p1x = extractSlot(ct_seg1, 0);
    auto ct_p1y = extractSlot(ct_seg1, 1);
    auto ct_p1z = extractSlot(ct_seg1, 2);
    auto ct_q1x = extractSlot(ct_seg1, 3);
    auto ct_q1y = extractSlot(ct_seg1, 4);
    auto ct_q1z = extractSlot(ct_seg1, 5);

    // Segment 2 (Bob) : P2=(x1,y1,z1), Q2=(x2,y2,z2)
    auto ct_p2x = extractSlot(ct_seg2, 0);
    auto ct_p2y = extractSlot(ct_seg2, 1);
    auto ct_p2z = extractSlot(ct_seg2, 2);
    auto ct_q2x = extractSlot(ct_seg2, 3);
    auto ct_q2y = extractSlot(ct_seg2, 4);
    auto ct_q2z = extractSlot(ct_seg2, 5);

    // ── Direction des segments sous FHE ───────────────────────────────────────
    // d1 = Q1 - P1
    auto ct_d1x = cc->EvalSub(ct_q1x, ct_p1x);
    auto ct_d1y = cc->EvalSub(ct_q1y, ct_p1y);
    auto ct_d1z = cc->EvalSub(ct_q1z, ct_p1z);

    // d2 = Q2 - P2
    auto ct_d2x = cc->EvalSub(ct_q2x, ct_p2x);
    auto ct_d2y = cc->EvalSub(ct_q2y, ct_p2y);
    auto ct_d2z = cc->EvalSub(ct_q2z, ct_p2z);

    // ── Produit vectoriel n = d1 x d2 sous FHE ───────────────────────────────
    // nx = d1y*d2z - d1z*d2y
    auto ct_nx = cc->EvalSub(
        cc->EvalMult(ct_d1y, ct_d2z),
        cc->EvalMult(ct_d1z, ct_d2y));
    // ny = d1z*d2x - d1x*d2z
    auto ct_ny = cc->EvalSub(
        cc->EvalMult(ct_d1z, ct_d2x),
        cc->EvalMult(ct_d1x, ct_d2z));
    // nz = d1x*d2y - d1y*d2x
    auto ct_nz = cc->EvalSub(
        cc->EvalMult(ct_d1x, ct_d2y),
        cc->EvalMult(ct_d1y, ct_d2x));

    // ── Produit mixte w = (P2-P1) . n sous FHE ───────────────────────────────
    // w = (p2x-p1x)*nx + (p2y-p1y)*ny + (p2z-p1z)*nz
    auto ct_wx = cc->EvalSub(ct_p2x, ct_p1x);
    auto ct_wy = cc->EvalSub(ct_p2y, ct_p1y);
    auto ct_wz = cc->EvalSub(ct_p2z, ct_p1z);

    auto ct_cop = cc->EvalAdd(
        cc->EvalAdd(
            cc->EvalMult(ct_wx, ct_nx),
            cc->EvalMult(ct_wy, ct_ny)),
        cc->EvalMult(ct_wz, ct_nz));

    // ── Test de coplanarite sous FHE (2 SS) ───────────────────────────────────
    auto ct_copOK = engine->isNearZeroBand(ct_cop, 1.0);
    bootstrapCount += 2;

    double copVal = engine->decryptValue(ct_copOK);
    if (copVal < 0.5) {
        // Pas coplanaires → pas de collision
        intersectionTests++;
        return 0.0;
    }

    // ── Orientations 2D sous FHE (projection sur plan XY) ────────────────────
    // o1 = orient(P1,Q1,P2) = d1y*(p2x-q1x) - d1x*(p2y-q1y)
    auto ct_o1 = cc->EvalSub(
        cc->EvalMult(ct_d1y, cc->EvalSub(ct_p2x, ct_q1x)),
        cc->EvalMult(ct_d1x, cc->EvalSub(ct_p2y, ct_q1y)));

    // o2 = orient(P1,Q1,Q2) = d1y*(q2x-q1x) - d1x*(q2y-q1y)
    auto ct_o2 = cc->EvalSub(
        cc->EvalMult(ct_d1y, cc->EvalSub(ct_q2x, ct_q1x)),
        cc->EvalMult(ct_d1x, cc->EvalSub(ct_q2y, ct_q1y)));

    // o3 = orient(P2,Q2,P1) = d2y*(p1x-q2x) - d2x*(p1y-q2y)
    auto ct_o3 = cc->EvalSub(
        cc->EvalMult(ct_d2y, cc->EvalSub(ct_p1x, ct_q2x)),
        cc->EvalMult(ct_d2x, cc->EvalSub(ct_p1y, ct_q2y)));

    // o4 = orient(P2,Q2,Q1) = d2y*(q1x-q2x) - d2x*(q1y-q2y)
    auto ct_o4 = cc->EvalSub(
        cc->EvalMult(ct_d2y, cc->EvalSub(ct_q1x, ct_q2x)),
        cc->EvalMult(ct_d2x, cc->EvalSub(ct_q1y, ct_q2y)));

    // ── Extraction des signes (4 SS) ──────────────────────────────────────────
    auto ct_s1 = engine->compareGreaterThanZero(ct_o1);
    auto ct_s2 = engine->compareGreaterThanZero(ct_o2);
    auto ct_s3 = engine->compareGreaterThanZero(ct_o3);
    auto ct_s4 = engine->compareGreaterThanZero(ct_o4);
    bootstrapCount += 4;
    signExtractions += 4;

    // ── Logique booleenne : XOR(s1,s2) AND XOR(s3,s4) ────────────────────────
    auto ct_xor12 = engine->eXor(ct_s1, ct_s2);
    auto ct_xor34 = engine->eXor(ct_s3, ct_s4);
    auto ct_inter = engine->eAnd(ct_xor12, ct_xor34);

    double result = engine->decryptValue(ct_inter);
    intersectionTests++;
    return result;
}

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
    auto result = engine->decryptVector(availability);
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

