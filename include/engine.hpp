#ifndef ENGINE_HPP
#define ENGINE_HPP

#include "openfhe.h"
#include "binfhecontext.h"
#include "types.hpp"
#include <vector>
#include <memory>
#include <string>
#include <iostream>

using namespace lbcrypto;

class CryptoEngine {
public:
    using CryptoContextCKKS = CryptoContext<DCRTPoly>;
    using CiphertextCKKS    = Ciphertext<DCRTPoly>;
    using PlaintextCKKS     = Plaintext;
    using KeyPairCKKS       = KeyPair<DCRTPoly>;

    struct Config {
        uint32_t      multDepth;
        uint32_t      scaleModSize;
        uint32_t      batchSize;
        SecurityLevel secLevel;
        uint32_t      ringDim;
        
        BINFHE_PARAMSET slBin;
        uint32_t      logQ_ccLWE;
        double        scaleSign;
        double        guardGain;
        uint32_t      switchValues;

        Config()
            : multDepth(18),
              scaleModSize(50),
              batchSize(1),
              secLevel(HEStd_NotSet),
              ringDim(8192),
              slBin(TOY),
              logQ_ccLWE(21),
              scaleSign(64.0),
              guardGain(1e4),
              switchValues(4) {}
    };

    CryptoEngine();
    ~CryptoEngine() = default;

    // Init / state
    void   initialize(const Config& cfg = Config());
    bool   isInitialized()    const { return initialized; }
    
    void   setupSchemeSwitching();
    bool   isSwitchingReady() const { return switchingReady; }

    // I/O
    CiphertextCKKS encryptValue(double value);
    CiphertextCKKS encryptVector(const std::vector<double>& v);
    std::vector<double> decryptVector(const CiphertextCKKS& ct);
    double             decryptValue(const CiphertextCKKS& ct);

    // Arithmetic
    CiphertextCKKS add   (const CiphertextCKKS& a, const CiphertextCKKS& b);
    CiphertextCKKS sub   (const CiphertextCKKS& a, const CiphertextCKKS& b);
    CiphertextCKKS mult  (const CiphertextCKKS& a, const CiphertextCKKS& b);
    CiphertextCKKS negate(const CiphertextCKKS& a);
    
    // Rotation and reduction
    CiphertextCKKS rotate(const CiphertextCKKS& x, int32_t index);
    CiphertextCKKS sum4(const CiphertextCKKS& x);
    CiphertextCKKS sumAll(const CiphertextCKKS& x);
    CiphertextCKKS reduce4ToSlot0(const CiphertextCKKS& x);  // NOUVEAU

    // Comparisons (scalar)
    CiphertextCKKS compareGreaterThanZero(const CiphertextCKKS& x);
    CiphertextCKKS compareValues(const CiphertextCKKS& a, const CiphertextCKKS& b);

    // Comparisons (batched)
    CiphertextCKKS compareGtZeroPacked(const CiphertextCKKS& xPacked, uint32_t kSlots);
    
    std::pair<CiphertextCKKS, CiphertextCKKS>
    compareWithGuardPacked(const CiphertextCKKS& xPacked, double tau, uint32_t kSlots);
    
    CiphertextCKKS isZeroWithGuardPackedScaled(
        const CiphertextCKKS& xPacked, double tau, double gain, uint32_t kSlots);
    
    // Guard band comparisons (scalar)
    std::pair<CiphertextCKKS, CiphertextCKKS>
    compareWithGuard(const CiphertextCKKS& x, double tau);
    
    CiphertextCKKS compareGEWithGuard(const CiphertextCKKS& a,
                                      const CiphertextCKKS& b, double tau);
    
    CiphertextCKKS isZeroWithGuard(const CiphertextCKKS& x, double tau);
    CiphertextCKKS isZeroWithGuardScaled(const CiphertextCKKS& x, double tau, double gain);

    // Debug helpers
    void debugDump(const char* tag, const CiphertextCKKS& x, int k = 4);  // NOUVEAU

    // Info / debug
    void   printParameters() const;
    size_t getSlotCount()    const { return config.batchSize; }
    void   printNoise(const CiphertextCKKS&, const std::string& label = "") const;
    
    CryptoContextCKKS getCKKSContext() const { return cc; }
    const KeyPairCKKS& getKeys()       const { return keys; }
    const Config& getConfig()          const { return config; }

    uint32_t getCompareSlots() const { return config.switchValues; }


    // Constantes alignées
    CiphertextCKKS constLike(const CiphertextCKKS& ref, double c);
    CiphertextCKKS oneLike(const CiphertextCKKS& ref);

    // Opérateurs logiques booléens (sur bits approx {0,1})
    CiphertextCKKS eNot(const CiphertextCKKS& a);
    CiphertextCKKS eAnd(const CiphertextCKKS& a, const CiphertextCKKS& b);
    CiphertextCKKS eOr(const CiphertextCKKS& a, const CiphertextCKKS& b);
    CiphertextCKKS eXor(const CiphertextCKKS& a, const CiphertextCKKS& b);

    // Comparaison <= 0
    CiphertextCKKS compareLEZero(const CiphertextCKKS& x);

    // Near-zero avec carré
    CiphertextCKKS isNearZeroSquared(const CiphertextCKKS& x, double tau);

    // Compare x <= threshold
    CiphertextCKKS compareLE(const CiphertextCKKS& x, double thr);

    // Comparaisons supplémentaires
    CiphertextCKKS compareGEZero(const CiphertextCKKS& x);
    CiphertextCKKS isNearZeroBand(const CiphertextCKKS& x, double tau);
    CiphertextCKKS betweenWithTol(const CiphertextCKKS& x, const CiphertextCKKS& a, 
                                const CiphertextCKKS& b, double eps);
    
    // On-segment par projection et colinéarité
    CiphertextCKKS onSegmentHE_full(const IntPoint& p, const IntPoint& q, 
                                    const IntPoint& r, double tauOri, double epsProj);
    
    // Comparaisons strictes et non-strictes
    CiphertextCKKS compareGT(const CiphertextCKKS& a, const CiphertextCKKS& b);
    CiphertextCKKS compareGE(const CiphertextCKKS& a, const CiphertextCKKS& b);
    CiphertextCKKS compareLE(const CiphertextCKKS& a, const CiphertextCKKS& b);

    // Comparaisons dérivées de compareGreaterThanZero
    CiphertextCKKS ltZero(const CiphertextCKKS& x);
    CiphertextCKKS geZero(const CiphertextCKKS& x);
    CiphertextCKKS gt(const CiphertextCKKS& a, const CiphertextCKKS& b);
    CiphertextCKKS ge(const CiphertextCKKS& a, const CiphertextCKKS& b);

    void testComparePolarity();
    
private:
    CryptoContextCKKS cc;
    KeyPairCKKS       keys;
    Config            config;
    bool              initialized     = false;
    bool              switchingReady  = false;
    LWEPrivateKey     lweSK;
    uint32_t          pLWE_ = 0;

    void checkInitialized()    const;
    void checkSwitchingReady() const;
    
    static uint32_t towersOf(const CiphertextCKKS& ct);
    static void     dbgCt(const char* tag, const CiphertextCKKS& ct);
    void            dbgContextTowers() const;

    CiphertextCKKS rawCompareZero(const CiphertextCKKS& x);
};

#endif // ENGINE_HPP