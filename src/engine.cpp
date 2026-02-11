#include "engine.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

CryptoEngine::CryptoEngine() : initialized(false), switchingReady(false), pLWE_(0) {}

void CryptoEngine::initialize(const Config& cfg) {
    config = cfg;

    CCParams<CryptoContextCKKSRNS> params;
    // === Paramètres fixes "exemple OpenFHE" ===
    params.SetMultiplicativeDepth(17);
    params.SetScalingModSize(50);
    params.SetFirstModSize(60);
    params.SetScalingTechnique(FLEXIBLEAUTO);
    params.SetSecurityLevel(HEStd_NotSet);   // sécurité gérée par ringDim fixé
    params.SetRingDim(8192);
    params.SetBatchSize(1);                  // *** pas de batching ***
    params.SetSecretKeyDist(UNIFORM_TERNARY);
    params.SetKeySwitchTechnique(HYBRID);
    params.SetNumLargeDigits(3);

    // Override config values with fixed ones
    config.multDepth = 17;
    config.scaleModSize = 50;
    config.batchSize = 1;
    config.ringDim = 8192;

    cc = GenCryptoContext(params);

    cc->Enable(PKE);
    cc->Enable(KEYSWITCH);
    cc->Enable(LEVELEDSHE);
    cc->Enable(ADVANCEDSHE);
    cc->Enable(SCHEMESWITCH);

    keys = cc->KeyGen();
    cc->EvalMultKeyGen(keys.secretKey);

    // *** AUCUNE clé de rotation à générer en mode scalaire ***

    initialized = true;

    std::cout << "CryptoEngine initialized with switching-ready context (scalar mode)\n";
    printParameters();
}

void CryptoEngine::setupSchemeSwitching() {
    checkInitialized();

    try {
        SchSwchParams swp;
        swp.SetSecurityLevelCKKS(HEStd_NotSet);
        swp.SetSecurityLevelFHEW(config.slBin);

        if (config.logQ_ccLWE < 23) {
            std::cout << "[CE] Adjusting logQ_ccLWE to 21 (minimum)\n";
            config.logQ_ccLWE = 23;
        }
        swp.SetCtxtModSizeFHEWLargePrec(config.logQ_ccLWE);

        // *** scalaire ***
        swp.SetNumSlotsCKKS(1);
        swp.SetNumValues(1);

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
        std::cout << "Scheme switching setup complete (scalar mode)\n";
        std::cout << "[CE] pLWE=" << pLWE
                  << ", scaleSign=" << config.scaleSign
                  << ", guardGain=" << config.guardGain << "\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error setting up scheme switching: " << e.what() << std::endl;
        throw;
    }
}
// Helpers de comparaison basés uniquement sur compareGreaterThanZero
CryptoEngine::CiphertextCKKS CryptoEngine::ltZero(const CiphertextCKKS& x) {
    // 1 si x < 0
    return compareGreaterThanZero(cc->EvalNegate(x));
}

CryptoEngine::CiphertextCKKS CryptoEngine::geZero(const CiphertextCKKS& x) {
    // 1 si x >= 0
    return cc->EvalSub(oneLike(x), ltZero(x));  // NOT(x<0)
}

CryptoEngine::CiphertextCKKS CryptoEngine::gt(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    // 1 si a > b
    return compareGreaterThanZero(cc->EvalSub(a, b));
}

CryptoEngine::CiphertextCKKS CryptoEngine::ge(const CiphertextCKKS& a, const CiphertextCKKS& b) {
    // 1 si a >= b
    return cc->EvalSub(oneLike(a), compareGreaterThanZero(cc->EvalSub(b, a)));
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareGE(const CiphertextCKKS& a,
                                                     const CiphertextCKKS& b) {
    auto gt = compareGT(b, a);  // b > a
    return cc->EvalSub(oneLike(gt), gt);  // NOT(b > a)
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareGT(const CiphertextCKKS& a,
                                                     const CiphertextCKKS& b) {
    checkSwitchingReady();
    // diff = a - b ; EvalCompare(0, diff) => 1 si diff > 0
    auto diff = cc->EvalSub(a, b);
    auto zero = cc->EvalSub(diff, diff);
    // PAS D'INVERSION : le résultat est déjà correct
    return cc->EvalCompareSchemeSwitching(zero, diff, 1, 1);
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareValues(const CiphertextCKKS& a, 
                                                         const CiphertextCKKS& b) {
    return compareGE(a, b);  // 1 si a >= b
}

// x >= 0 ? (1 si x >= 0) -- tout chiffré
CryptoEngine::CiphertextCKKS CryptoEngine::compareGEZero(const CiphertextCKKS& x) {
    checkSwitchingReady();
    auto zero = constLike(x, 0.0);
    // compareValues(x, zero) renvoie 1 si x >= 0 (après correction de l'inversion)
    return compareValues(x, zero);
}

// |x| <= tau ? via NOT( (x - tau) > 0 OR (-x - tau) > 0 )
CryptoEngine::CiphertextCKKS CryptoEngine::isNearZeroBand(const CiphertextCKKS& x, double tau) {
    checkSwitchingReady();

    auto x_minus_tau = cc->EvalSub(x, constLike(x, tau));
    auto gt_pos = compareGreaterThanZero(x_minus_tau); // 1 si x - tau > 0

    auto minusx_minus_tau = cc->EvalSub(cc->EvalNegate(x), constLike(x, tau));
    auto gt_neg = compareGreaterThanZero(minusx_minus_tau); // 1 si -x - tau > 0

    auto one = oneLike(gt_pos);
    // outside = gt_pos OR gt_neg = 1 - (1-gt_pos)*(1-gt_neg)
    auto outside = cc->EvalSub(one, cc->EvalMult(cc->EvalSub(one, gt_pos), cc->EvalSub(one, gt_neg)));
    // nearZero = NOT(outside)
    return cc->EvalSub(one, outside); // 1 si |x| <= tau
}

// [a, b] avec tolérance eps : (x >= a-eps) AND (x <= b+eps)
CryptoEngine::CiphertextCKKS CryptoEngine::betweenWithTol(
    const CiphertextCKKS& x, const CiphertextCKKS& a, const CiphertextCKKS& b, double eps)
{
    checkSwitchingReady();
    auto a_lo = cc->EvalSub(a, constLike(a, eps));      // a - eps
    auto b_hi = cc->EvalAdd(b, constLike(b, eps));      // b + eps
    // ge = (x >= a-eps) ; le = (x <= b+eps)
    auto ge = compareValues(x, a_lo);  // x >= a_lo
    auto le = compareValues(b_hi, x);  // b_hi >= x donc x <= b_hi
    return cc->EvalMult(ge, le); // AND
}

// Helper "x <= thr ?" -> 1 si x <= thr
CryptoEngine::CiphertextCKKS CryptoEngine::compareLE(const CiphertextCKKS& a,
                                                     const CiphertextCKKS& b) {
    auto gt = compareGT(a, b);  // a > b
    return cc->EvalSub(oneLike(gt), gt);  // NOT(a > b)
}

// --- Constantes alignées au niveau "ref" ---
CryptoEngine::CiphertextCKKS CryptoEngine::constLike(const CiphertextCKKS& ref, double c) {
    checkInitialized();
    auto z = cc->EvalSub(ref, ref);  // 0 au même niveau
    return cc->EvalAdd(z, c);
}

CryptoEngine::CiphertextCKKS CryptoEngine::oneLike(const CiphertextCKKS& ref) {
    return constLike(ref, 1.0);
}

// --- Logique booléenne approx. en CKKS (a,b ~ {0,1}) ---
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
    // a + b - 2ab
    auto two = constLike(a, 2.0);
    return cc->EvalAdd(cc->EvalAdd(a, b), cc->EvalNegate(cc->EvalMult(two, cc->EvalMult(a, b))));
}

// Near-zero avec carré (corrigé automatiquement par compareValues)
CryptoEngine::CiphertextCKKS CryptoEngine::isNearZeroSquared(const CiphertextCKKS& x, double tau) {
    checkSwitchingReady();
    auto xsq = cc->EvalMult(x, x);                     // x^2
    auto thr = constLike(xsq, tau * tau);              // τ^2 aligné
    return compareValues(thr, xsq);                    // Maintenant correct : 1 si x^2 <= τ^2
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareGreaterThanZero(const CiphertextCKKS& x) {
    checkSwitchingReady();
    auto zero = cc->EvalSub(x, x);
    // EvalCompare(0, x) -> 1 si x > 0. PAS D'INVERSION.
    return cc->EvalCompareSchemeSwitching(zero, x, 1, 1);
}

CryptoEngine::CiphertextCKKS CryptoEngine::compareLEZero(const CiphertextCKKS& x) {
    checkSwitchingReady();
    auto zero = cc->EvalSub(x, x);
    return compareLE(x, zero); // 1 si x <= 0
}

// I/O Operations
CryptoEngine::CiphertextCKKS CryptoEngine::encryptValue(double value) {
    checkInitialized();
    std::vector<double> vec(1, value);
    auto pt = cc->MakeCKKSPackedPlaintext(vec);
    return cc->Encrypt(keys.publicKey, pt);
}

CryptoEngine::CiphertextCKKS CryptoEngine::encryptVector(const std::vector<double>& v) {
    checkInitialized();
    std::vector<double> buf(1, v.empty() ? 0.0 : v[0]);
    auto pt = cc->MakeCKKSPackedPlaintext(buf);
    return cc->Encrypt(keys.publicKey, pt);
}

double CryptoEngine::decryptValue(const CiphertextCKKS& ct) {
    checkInitialized();
    Plaintext pt;
    cc->Decrypt(keys.secretKey, ct, &pt);
    auto cv = pt->GetCKKSPackedValue();
    return cv.empty() ? 0.0 : cv[0].real();
}

std::vector<double> CryptoEngine::decryptVector(const CiphertextCKKS& ct) {
    return {decryptValue(ct)};
}

// Arithmetic Operations
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

// Guard band comparison (scalar)
CryptoEngine::CiphertextCKKS
CryptoEngine::isZeroWithGuardScaled(const CiphertextCKKS& x, double tau, double gain) {
    checkSwitchingReady();

    // xScaled = x * gain
    std::vector<double> gvec(1, gain);
    auto ptGain = cc->MakeCKKSPackedPlaintext(gvec);
    auto xScaled = cc->EvalMult(x, ptGain);

    // y_plus  = xScaled - (+tau*gain)
    // y_minus = xScaled - (-tau*gain)
    std::vector<double> tpos(1,  tau * gain);
    std::vector<double> tneg(1, -tau * gain);
    auto ptTP = cc->MakeCKKSPackedPlaintext(tpos);
    auto ptTN = cc->MakeCKKSPackedPlaintext(tneg);

    auto y_plus  = cc->EvalSub(xScaled, ptTP);
    auto y_minus = cc->EvalSub(xScaled, ptTN);

    // z = 0 au niveau des y_*
    auto zP = cc->EvalSub(y_plus,  y_plus);
    auto zM = cc->EvalSub(y_minus, y_minus);

    auto gtPos = cc->EvalCompareSchemeSwitching(zP, y_plus,  1, 1);
    auto gtNeg = cc->EvalCompareSchemeSwitching(zM, y_minus, 1, 1);

    // one aligné au niveau de gtPos
    auto one = cc->EvalSub(gtPos, gtPos);
    one = cc->EvalAdd(one, 1.0);

    auto not_gtPos = cc->EvalSub(one, gtPos);
    return cc->EvalMult(not_gtPos, gtNeg); // 1 si |x|<=tau
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
    std::cout << "\n=== CKKS Parameters (Scalar Mode) ===\n";
    std::cout << "Ring dimension: " << cc->GetRingDimension() << "\n";
    std::cout << "Multiplicative depth: " << config.multDepth << "\n";
    std::cout << "Scale mod size: " << config.scaleModSize << " bits\n";
    std::cout << "Batch size: " << config.batchSize << "\n";
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