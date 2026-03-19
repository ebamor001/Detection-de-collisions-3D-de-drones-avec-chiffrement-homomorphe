#include "engine.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

CryptoEngine::CryptoEngine() : initialized(false), switchingReady(false), pLWE_(0) {}

void CryptoEngine::initialize(const Config& cfg) {
    config = cfg;

    CCParams<CryptoContextCKKSRNS> params;

    // -- AVANT BATCHING --
    // params.SetMultiplicativeDepth(17);
    // -- APRES BATCHING --
    // 17 -> 20 : marge pour le cross product 3D (+3 multiplications)
    params.SetMultiplicativeDepth(20);

    params.SetScalingModSize(50);
    params.SetFirstModSize(60);
    params.SetScalingTechnique(FLEXIBLEAUTO);
    params.SetSecurityLevel(HEStd_NotSet);
    params.SetRingDim(8192);

    // -- AVANT BATCHING --
    // params.SetBatchSize(1);                  // 1 seule valeur par ciphertext
    // -- APRES BATCHING --
    // config.batchSize = 4096 (ringDim/2 = nombre max de slots)
    // Le cout de chaque operation est IDENTIQUE que batchSize soit 1 ou 4096.
    params.SetBatchSize(config.batchSize);

    params.SetSecretKeyDist(UNIFORM_TERNARY);
    params.SetKeySwitchTechnique(HYBRID);
    params.SetNumLargeDigits(3);

    // -- AVANT BATCHING --
    // config.multDepth = 17;
    // config.scaleModSize = 50;
    // config.batchSize = 1;
    // config.ringDim = 8192;
    // -- APRES BATCHING --
    config.multDepth    = 20;
    config.scaleModSize = 50;
    config.ringDim      = 8192;

    cc = GenCryptoContext(params);

    cc->Enable(PKE);
    cc->Enable(KEYSWITCH);
    cc->Enable(LEVELEDSHE);
    cc->Enable(ADVANCEDSHE);
    cc->Enable(SCHEMESWITCH);

    keys = cc->KeyGen();
    cc->EvalMultKeyGen(keys.secretKey);

    // -- AVANT BATCHING --
    // (rien - aucune cle de rotation en mode scalaire)
    // -- APRES BATCHING --
    // Cles de rotation {1,-1,2,-2,...,2048,-2048} pour rotate(), sum4(), sumAll()
    if (config.batchSize > 1) {
        std::vector<int32_t> rotIndices;
        for (int32_t i = 1; i < (int32_t)config.batchSize; i *= 2) {
            rotIndices.push_back(i);
            rotIndices.push_back(-i);
        }
        cc->EvalRotateKeyGen(keys.secretKey, rotIndices);
        std::cout << "[CE] Rotation keys generated for "
                  << rotIndices.size() << " indices\n";
    }

    initialized = true;

    // -- AVANT BATCHING --
    // std::cout << "CryptoEngine initialized with switching-ready context (scalar mode)\n";
    // -- APRES BATCHING --
    std::cout << "CryptoEngine initialized (batch mode, "
              << config.batchSize << " slots)\n";
    printParameters();
}

void CryptoEngine::setupSchemeSwitching() {
    checkInitialized();

    try {
        SchSwchParams swp;
        swp.SetSecurityLevelCKKS(HEStd_NotSet);
        swp.SetSecurityLevelFHEW(config.slBin);

        if (config.logQ_ccLWE < 23) {
            std::cout << "[CE] Adjusting logQ_ccLWE to 23 (minimum)\n";
            config.logQ_ccLWE = 23;
        }
        swp.SetCtxtModSizeFHEWLargePrec(config.logQ_ccLWE);

        // -- AVANT BATCHING --
        // swp.SetNumSlotsCKKS(1);   // 1 seul slot par scheme switch
        // swp.SetNumValues(1);
        // -- APRES BATCHING --
        // N slots convertis en UN seul appel = gain x150
        uint32_t numSlots = config.switchValues;
        swp.SetNumSlotsCKKS(numSlots);
        swp.SetNumValues(numSlots);

        lweSK = cc->EvalSchemeSwitchingSetup(swp);
        cc->EvalSchemeSwitchingKeyGen(keys, lweSK);
        cc->EvalCKKStoFHEWPrecompute(static_cast<double>(config.scaleSign));

        auto ccLWE = cc->GetBinCCForSchemeSwitch();
        const uint32_t L = config.logQ_ccLWE;
        auto modulusLWE = 1u << L;
        auto beta = ccLWE->GetBeta().ConvertToInt();
        auto pLWE = modulusLWE / (2u * beta);
        pLWE_ = pLWE;

        cc->EvalCompareSwitchPrecompute(pLWE, config.scaleSign);

        switchingReady = true;
        // -- AVANT BATCHING --
        // std::cout << "Scheme switching setup complete (scalar mode)\n";
        // -- APRES BATCHING --
        std::cout << "Scheme switching setup complete (batch mode, "
                  << numSlots << " slots)\n";
        std::cout << "[CE] pLWE=" << pLWE
                  << ", scaleSign=" << config.scaleSign
                  << ", guardGain=" << config.guardGain
                  << ", switchSlots=" << numSlots << "\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error setting up scheme switching: " << e.what() << std::endl;
        throw;
    }
}

// Comparaisons derivees - INCHANGE (appellent compareGreaterThanZero qui est batche)
CryptoEngine::CiphertextCKKS CryptoEngine::ltZero(const CiphertextCKKS& x) {
    return compareGreaterThanZero(cc->EvalNegate(x));
}

CryptoEngine::CiphertextCKKS CryptoEngine::geZero(const CiphertextCKKS& x) {
    return cc->EvalSub(oneLike(x), ltZero(x));
}

CryptoEngine::CiphertextCKKS CryptoEngine::gt(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    return compareGreaterThanZero(cc->EvalSub(a, b));
}

CryptoEngine::CiphertextCKKS CryptoEngine::ge(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    return cc->EvalSub(oneLike(a), compareGreaterThanZero(cc->EvalSub(b, a)));
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareGE(const CiphertextCKKS& a,
                                                     const CiphertextCKKS& b) {
    auto gtResult = compareGT(b, a);
    return cc->EvalSub(oneLike(gtResult), gtResult);
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareGT(const CiphertextCKKS& a,
                                                     const CiphertextCKKS& b) {
    checkSwitchingReady();
    auto diff = cc->EvalSub(a, b);
    auto zero = cc->EvalSub(diff, diff);
    // -- AVANT BATCHING --
    // return cc->EvalCompareSchemeSwitching(zero, diff, 1, 1);
    // -- APRES BATCHING --
    uint32_t numSlots = config.switchValues;
    return cc->EvalCompareSchemeSwitching(zero, diff, numSlots, numSlots);
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareValues(const CiphertextCKKS& a,
                                                         const CiphertextCKKS& b) {
    return compareGE(a, b);
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareGEZero(const CiphertextCKKS& x) {
    checkSwitchingReady();
    auto zero = constLike(x, 0.0);
    return compareValues(x, zero);
}

// INCHANGE
CryptoEngine::CiphertextCKKS CryptoEngine::isNearZeroBand(const CiphertextCKKS& x, double tau) {
    checkSwitchingReady();
    auto x_minus_tau = cc->EvalSub(x, constLike(x, tau));
    auto gt_pos = compareGreaterThanZero(x_minus_tau);
    auto minusx_minus_tau = cc->EvalSub(cc->EvalNegate(x), constLike(x, tau));
    auto gt_neg = compareGreaterThanZero(minusx_minus_tau);
    auto one = oneLike(gt_pos);
    auto outside = cc->EvalSub(one, cc->EvalMult(cc->EvalSub(one, gt_pos), cc->EvalSub(one, gt_neg)));
    return cc->EvalSub(one, outside);
}

CryptoEngine::CiphertextCKKS CryptoEngine::betweenWithTol(
    const CiphertextCKKS& x, const CiphertextCKKS& a, const CiphertextCKKS& b, double eps) {
    checkSwitchingReady();
    auto a_lo = cc->EvalSub(a, constLike(a, eps));
    auto b_hi = cc->EvalAdd(b, constLike(b, eps));
    auto geResult = compareValues(x, a_lo);
    auto leResult = compareValues(b_hi, x);
    return cc->EvalMult(geResult, leResult);
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareLE(const CiphertextCKKS& a,
                                                     const CiphertextCKKS& b) {
    auto gtResult = compareGT(a, b);
    return cc->EvalSub(oneLike(gtResult), gtResult);
}

CryptoEngine::CiphertextCKKS CryptoEngine::constLike(const CiphertextCKKS& ref, double c) {
    checkInitialized();
    auto z = cc->EvalSub(ref, ref);
    return cc->EvalAdd(z, c);
}

CryptoEngine::CiphertextCKKS CryptoEngine::oneLike(const CiphertextCKKS& ref) {
    return constLike(ref, 1.0);
}

CryptoEngine::CiphertextCKKS CryptoEngine::eNot(const CiphertextCKKS& a) {
    checkInitialized();
    return cc->EvalSub(oneLike(a), a);
}

CryptoEngine::CiphertextCKKS CryptoEngine::eAnd(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    checkInitialized();
    return cc->EvalMult(a, b);
}

CryptoEngine::CiphertextCKKS CryptoEngine::eOr(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    checkInitialized();
    auto one = oneLike(a);
    return cc->EvalSub(one, cc->EvalMult(cc->EvalSub(one, a), cc->EvalSub(one, b)));
}

CryptoEngine::CiphertextCKKS CryptoEngine::eXor(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    checkInitialized();
    auto two = constLike(a, 2.0);
    return cc->EvalAdd(cc->EvalAdd(a, b), cc->EvalNegate(cc->EvalMult(two, cc->EvalMult(a, b))));
}

CryptoEngine::CiphertextCKKS CryptoEngine::isNearZeroSquared(const CiphertextCKKS& x, double tau) {
    checkSwitchingReady();
    auto xsq = cc->EvalMult(x, x);
    auto thr = constLike(xsq, tau * tau);
    return compareValues(thr, xsq);
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareGreaterThanZero(const CiphertextCKKS& x) {
    checkSwitchingReady();
    auto zero = cc->EvalSub(x, x);
    // -- AVANT BATCHING --
    // return cc->EvalCompareSchemeSwitching(zero, x, 1, 1);
    // -- APRES BATCHING --
    uint32_t numSlots = config.switchValues;
    return cc->EvalCompareSchemeSwitching(zero, x, numSlots, numSlots);
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareLEZero(const CiphertextCKKS& x) {
    checkSwitchingReady();
    auto zero = cc->EvalSub(x, x);
    return compareLE(x, zero);
}

// I/O

CryptoEngine::CiphertextCKKS CryptoEngine::encryptValue(double value) {
    checkInitialized();
    std::vector<double> vec(1, value);
    auto pt = cc->MakeCKKSPackedPlaintext(vec);
    return cc->Encrypt(keys.publicKey, pt);
}

// -- AVANT BATCHING --
// CryptoEngine::CiphertextCKKS CryptoEngine::encryptVector(const std::vector<double>& v) {
//     checkInitialized();
//     std::vector<double> buf(1, v.empty() ? 0.0 : v[0]);  // BUG: ne prend que v[0]
//     auto pt = cc->MakeCKKSPackedPlaintext(buf);
//     return cc->Encrypt(keys.publicKey, pt);
// }
// -- APRES BATCHING --
// Passe le vecteur COMPLET. encryptVector({1,2,3,4}) -> slots [1,2,3,4,0,...,0]
CryptoEngine::CiphertextCKKS CryptoEngine::encryptVector(const std::vector<double>& v) {
    checkInitialized();
    if (v.empty()) {
        return encryptValue(0.0);
    }
    auto pt = cc->MakeCKKSPackedPlaintext(v);
    return cc->Encrypt(keys.publicKey, pt);
}

double CryptoEngine::decryptValue(const CiphertextCKKS& ct) {
    checkInitialized();
    Plaintext pt;
    cc->Decrypt(keys.secretKey, ct, &pt);
    auto cv = pt->GetCKKSPackedValue();
    return cv.empty() ? 0.0 : cv[0].real();
}

// -- AVANT BATCHING --
// std::vector<double> CryptoEngine::decryptVector(const CiphertextCKKS& ct) {
//     return {decryptValue(ct)};  // BUG: retourne toujours taille 1
// }
// -- APRES BATCHING --
// Lit les k premiers slots. Signature changee: parametre k ajoute.
std::vector<double> CryptoEngine::decryptVector(const CiphertextCKKS& ct, uint32_t k) {
    checkInitialized();
    Plaintext pt;
    cc->Decrypt(keys.secretKey, ct, &pt);
    auto cv = pt->GetCKKSPackedValue();
    if (k == 0) k = config.batchSize;
    uint32_t n = std::min(k, (uint32_t)cv.size());
    std::vector<double> result(n);
    for (uint32_t i = 0; i < n; i++) {
        result[i] = cv[i].real();
    }
    return result;
}

// Arithmetique - INCHANGE
CryptoEngine::CiphertextCKKS CryptoEngine::add(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    checkInitialized();
    return cc->EvalAdd(a, b);
}

CryptoEngine::CiphertextCKKS CryptoEngine::sub(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    checkInitialized();
    return cc->EvalSub(a, b);
}

CryptoEngine::CiphertextCKKS CryptoEngine::mult(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    checkInitialized();
    return cc->EvalMult(a, b);
}

CryptoEngine::CiphertextCKKS CryptoEngine::negate(const CiphertextCKKS& a) {
    checkInitialized();
    return cc->EvalNegate(a);
}

// =========================================================
// NOUVEAU - Rotations (n'existait pas avant batching)
// =========================================================

CryptoEngine::CiphertextCKKS CryptoEngine::rotate(const CiphertextCKKS& x, int32_t index) {
    checkInitialized();
    return cc->EvalRotate(x, index);
}

CryptoEngine::CiphertextCKKS CryptoEngine::sum4(const CiphertextCKKS& x) {
    checkInitialized();
    auto r2 = cc->EvalRotate(x, 2);
    auto s1 = cc->EvalAdd(x, r2);
    auto r1 = cc->EvalRotate(s1, 1);
    return cc->EvalAdd(s1, r1);
}

CryptoEngine::CiphertextCKKS CryptoEngine::sumAll(const CiphertextCKKS& x) {
    checkInitialized();
    auto result = x;
    for (uint32_t step = 1; step < config.batchSize; step *= 2) {
        auto rotated = cc->EvalRotate(result, step);
        result = cc->EvalAdd(result, rotated);
    }
    return result;
}

CryptoEngine::CiphertextCKKS CryptoEngine::reduce4ToSlot0(const CiphertextCKKS& x) {
    return sum4(x);
}

// =========================================================
// NOUVEAU - Comparaison batchee avec kSlots explicite
// =========================================================

CryptoEngine::CiphertextCKKS CryptoEngine::compareGtZeroPacked(
    const CiphertextCKKS& xPacked, uint32_t kSlots) {
    checkSwitchingReady();
    auto zero = cc->EvalSub(xPacked, xPacked);
    return cc->EvalCompareSchemeSwitching(zero, xPacked, kSlots, kSlots);
}

// =========================================================
// NOUVEAU - Debug helper
// =========================================================

void CryptoEngine::debugDump(const char* tag, const CiphertextCKKS& x, int k) {
    auto vals = decryptVector(x, k);
    std::cout << "[DBG] " << tag << " : [";
    for (int i = 0; i < (int)vals.size(); i++) {
        if (i > 0) std::cout << ", ";
        std::cout << vals[i];
    }
    std::cout << "]" << std::endl;
}

// Guard band - EvalCompareSchemeSwitching utilise numSlots
CryptoEngine::CiphertextCKKS
CryptoEngine::isZeroWithGuardScaled(const CiphertextCKKS& x, double tau, double gain) {
    checkSwitchingReady();
    std::vector<double> gvec(1, gain);
    auto ptGain = cc->MakeCKKSPackedPlaintext(gvec);
    auto xScaled = cc->EvalMult(x, ptGain);
    std::vector<double> tpos(1,  tau * gain);
    std::vector<double> tneg(1, -tau * gain);
    auto ptTP = cc->MakeCKKSPackedPlaintext(tpos);
    auto ptTN = cc->MakeCKKSPackedPlaintext(tneg);
    auto y_plus  = cc->EvalSub(xScaled, ptTP);
    auto y_minus = cc->EvalSub(xScaled, ptTN);
    auto zP = cc->EvalSub(y_plus,  y_plus);
    auto zM = cc->EvalSub(y_minus, y_minus);
    // -- AVANT BATCHING --
    // auto gtPos = cc->EvalCompareSchemeSwitching(zP, y_plus,  1, 1);
    // auto gtNeg = cc->EvalCompareSchemeSwitching(zM, y_minus, 1, 1);
    // -- APRES BATCHING --
    uint32_t numSlots = config.switchValues;
    auto gtPos = cc->EvalCompareSchemeSwitching(zP, y_plus,  numSlots, numSlots);
    auto gtNeg = cc->EvalCompareSchemeSwitching(zM, y_minus, numSlots, numSlots);
    auto one = cc->EvalSub(gtPos, gtPos);
    one = cc->EvalAdd(one, 1.0);
    auto not_gtPos = cc->EvalSub(one, gtPos);
    return cc->EvalMult(not_gtPos, gtNeg);
}

CryptoEngine::CiphertextCKKS
CryptoEngine::isZeroWithGuard(const CiphertextCKKS& x, double tau) {
    return isZeroWithGuardScaled(x, tau, config.guardGain);
}

// Helpers
void CryptoEngine::printParameters() const {
    if (!initialized) {
        std::cout << "Engine not initialized\n";
        return;
    }
    // -- AVANT BATCHING --
    // std::cout << "\n=== CKKS Parameters (Scalar Mode) ===\n";
    // -- APRES BATCHING --
    std::cout << "\n=== CKKS Parameters (Batch Mode) ===\n";
    std::cout << "Ring dimension: " << cc->GetRingDimension() << "\n";
    std::cout << "Multiplicative depth: " << config.multDepth << "\n";
    std::cout << "Scale mod size: " << config.scaleModSize << " bits\n";
    std::cout << "Batch size: " << config.batchSize << " slots\n";
    std::cout << "Switch values: " << config.switchValues << " slots\n";
    std::cout << "Scale sign: " << config.scaleSign << "\n";
    std::cout << "Guard gain: " << config.guardGain << "\n";
}

void CryptoEngine::checkInitialized() const {
    if (!initialized)
        throw std::runtime_error("CryptoEngine not initialized");
}

void CryptoEngine::checkSwitchingReady() const {
    checkInitialized();
    if (!switchingReady)
        throw std::runtime_error("Scheme switching not ready");
}

void CryptoEngine::printNoise(const CiphertextCKKS&, const std::string& label) const {
    if (!initialized) return;
    std::cout << "Noise measurement";
    if (!label.empty()) std::cout << " [" << label << "]";
    std::cout << ": [disabled]\n";
}
