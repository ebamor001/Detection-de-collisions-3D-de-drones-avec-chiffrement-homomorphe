#ifndef ENGINE_SERIAL_HPP
#define ENGINE_SERIAL_HPP

#include "engine.hpp"
#include "openfhe.h"
#include "cryptocontext-ser.h"
#include "key/key-ser.h"
#include "scheme/ckksrns/ckksrns-ser.h"
#include "binfhe/binfhecontext-ser.h"
#include <sstream>
#include <fstream>
#include <cstring>

using namespace lbcrypto;

inline std::string serializeContext(CryptoContext<DCRTPoly> cc) {
    std::ostringstream oss;
    Serial::Serialize(cc, oss, SerType::BINARY);
    return oss.str();
}
inline std::string serializePublicKey(const PublicKey<DCRTPoly>& pk) {
    std::ostringstream oss;
    Serial::Serialize(pk, oss, SerType::BINARY);
    return oss.str();
}
inline std::string serializeEvalMultKeys(CryptoContext<DCRTPoly> cc) {
    std::ostringstream oss;
    cc->SerializeEvalMultKey(oss, SerType::BINARY);
    return oss.str();
}
inline std::string serializeEvalSumKeys(CryptoContext<DCRTPoly> cc) {
    std::ostringstream oss;
    cc->SerializeEvalAutomorphismKey(oss, SerType::BINARY);
    return oss.str();
}
inline std::string serializeCiphertext(const Ciphertext<DCRTPoly>& ct) {
    std::ostringstream oss;
    Serial::Serialize(ct, oss, SerType::BINARY);
    return oss.str();
}
inline CryptoContext<DCRTPoly> deserializeContext(const std::string& data) {
    CryptoContext<DCRTPoly> cc;
    std::istringstream iss(data);
    Serial::Deserialize(cc, iss, SerType::BINARY);
    return cc;
}
inline PublicKey<DCRTPoly> deserializePublicKey(const std::string& data) {
    PublicKey<DCRTPoly> pk;
    std::istringstream iss(data);
    Serial::Deserialize(pk, iss, SerType::BINARY);
    return pk;
}
inline void deserializeEvalMultKeys(CryptoContext<DCRTPoly> cc, const std::string& data) {
    std::istringstream iss(data);
    cc->DeserializeEvalMultKey(iss, SerType::BINARY);
}
inline void deserializeEvalSumKeys(CryptoContext<DCRTPoly> cc, const std::string& data) {
    std::istringstream iss(data);
    cc->DeserializeEvalAutomorphismKey(iss, SerType::BINARY);
}
inline Ciphertext<DCRTPoly> deserializeCiphertext(const std::string& data) {
    Ciphertext<DCRTPoly> ct;
    std::istringstream iss(data);
    Serial::Deserialize(ct, iss, SerType::BINARY);
    return ct;
}


inline std::string serializeSecretKey(const PrivateKey<DCRTPoly>& sk) {
    std::ostringstream oss;
    Serial::Serialize(sk, oss, SerType::BINARY);
    return oss.str();
}

inline PrivateKey<DCRTPoly> deserializeSecretKey(const std::string& data) {
    PrivateKey<DCRTPoly> sk;
    std::istringstream iss(data);
    Serial::Deserialize(sk, iss, SerType::BINARY);
    return sk;
}

// ── FHEW bootstrapping key serialization ──────────────────────────────────────
// BinFHEContext::save() only serializes params, NOT m_BTKey / m_BTKey_map.
// These must be serialized and transferred separately for modeDetect to work.

namespace detail {
    inline std::string packLen(const std::string& s) {
        std::string out(4 + s.size(), '\0');
        uint32_t len = static_cast<uint32_t>(s.size());
        std::memcpy(&out[0], &len, 4);
        std::memcpy(&out[4], s.data(), s.size());
        return out;
    }
    inline std::string readLen(const std::string& buf, size_t& off) {
        if (off + 4 > buf.size()) throw std::runtime_error("BTKey: buffer too short");
        uint32_t len;
        std::memcpy(&len, buf.data() + off, 4);
        off += 4;
        if (off + len > buf.size()) throw std::runtime_error("BTKey: data truncated");
        std::string s(buf.data() + off, len);
        off += len;
        return s;
    }
    template<typename T>
    inline std::string serOne(const T& v) {
        std::ostringstream oss;
        Serial::Serialize(v, oss, SerType::BINARY);
        return oss.str();
    }
    template<typename T>
    inline void deserOne(const std::string& data, T& v) {
        std::istringstream iss(data);
        Serial::Deserialize(v, iss, SerType::BINARY);
    }
}

inline std::string serializeBTKey(CryptoContext<DCRTPoly> cc) {
    using namespace detail;
    auto ccLWE = cc->GetBinCCForSchemeSwitch();
    if (!ccLWE) throw std::runtime_error("serializeBTKey: no LWE context");

    std::string result;

    // m_BTKey (used by GINX)
    auto bsk  = ccLWE->GetRefreshKey();
    auto ksk  = ccLWE->GetSwitchKey();
    result += packLen(bsk  ? serOne(bsk)  : "");
    result += packLen(ksk  ? serOne(ksk)  : "");

    // m_BTKey_map (used by LMKCDEY / scheme switching)
    auto keyMap = ccLWE->GetBTKeyMap();
    uint32_t n = static_cast<uint32_t>(keyMap->size());
    result.resize(result.size() + 4);
    std::memcpy(&result[result.size() - 4], &n, 4);

    for (auto& [baseG, key] : *keyMap) {
        result.resize(result.size() + 4);
        std::memcpy(&result[result.size() - 4], &baseG, 4);
        result += packLen(key.BSkey ? serOne(key.BSkey) : "");
        result += packLen(key.KSkey ? serOne(key.KSkey) : "");
    }
    return result;
}

inline void deserializeBTKeyInto(std::shared_ptr<lbcrypto::BinFHEContext> ccLWE, const std::string& data) {
    using namespace detail;
    if (!ccLWE) throw std::runtime_error("deserializeBTKeyInto: null LWE context");

    size_t off = 0;

    std::string bsk_data = readLen(data, off);
    std::string ksk_data = readLen(data, off);
    RingGSWACCKey bsk;
    LWESwitchingKey ksk;
    if (!bsk_data.empty()) deserOne(bsk_data, bsk);
    if (!ksk_data.empty()) deserOne(ksk_data, ksk);
    if (bsk || ksk) ccLWE->BTKeyLoad({bsk, ksk, nullptr});

    if (off + 4 > data.size()) return;
    uint32_t n;
    std::memcpy(&n, data.data() + off, 4); off += 4;
    for (uint32_t i = 0; i < n; ++i) {
        if (off + 4 > data.size()) break;
        uint32_t baseG;
        std::memcpy(&baseG, data.data() + off, 4); off += 4;
        std::string mb_data = readLen(data, off);
        std::string mk_data = readLen(data, off);
        RingGSWACCKey mb;
        LWESwitchingKey mk;
        if (!mb_data.empty()) deserOne(mb_data, mb);
        if (!mk_data.empty()) deserOne(mk_data, mk);
        ccLWE->BTKeyMapLoadSingleElement(baseG, {mb, mk, nullptr});
    }
}

inline void deserializeBTKey(CryptoContext<DCRTPoly> cc, const std::string& data) {
    using namespace detail;
    auto ccLWE = cc->GetBinCCForSchemeSwitch();
    if (!ccLWE) throw std::runtime_error("deserializeBTKey: no LWE context");

    size_t off = 0;

    // m_BTKey
    std::string bsk_data = readLen(data, off);
    std::string ksk_data = readLen(data, off);
    RingGSWACCKey bsk;
    LWESwitchingKey ksk;
    if (!bsk_data.empty()) deserOne(bsk_data, bsk);
    if (!ksk_data.empty()) deserOne(ksk_data, ksk);
    if (bsk || ksk) ccLWE->BTKeyLoad({bsk, ksk, nullptr});

    // m_BTKey_map
    if (off + 4 > data.size()) return;
    uint32_t n;
    std::memcpy(&n, data.data() + off, 4); off += 4;
    for (uint32_t i = 0; i < n; ++i) {
        if (off + 4 > data.size()) break;
        uint32_t baseG;
        std::memcpy(&baseG, data.data() + off, 4); off += 4;
        std::string mb_data = readLen(data, off);
        std::string mk_data = readLen(data, off);
        RingGSWACCKey mb;
        LWESwitchingKey mk;
        if (!mb_data.empty()) deserOne(mb_data, mb);
        if (!mk_data.empty()) deserOne(mk_data, mk);
        ccLWE->BTKeyMapLoadSingleElement(baseG, {mb, mk, nullptr});
    }
}
inline std::string serializeSwkFC(CryptoContext<DCRTPoly> cc) {
    auto swkFC = cc->GetSwkFC();
    if (!swkFC) {
        throw std::runtime_error("serializeSwkFC: swkFC is null");
    }
    std::ostringstream oss;
    Serial::Serialize(swkFC, oss, SerType::BINARY);
    return oss.str();
}

inline void deserializeSwkFC(CryptoContext<DCRTPoly> cc, const std::string& data) {
    if (!cc) {
        throw std::runtime_error("deserializeSwkFC: crypto context is null");
    }
    Ciphertext<DCRTPoly> swkFC;
    std::istringstream iss(data);
    Serial::Deserialize(swkFC, iss, SerType::BINARY);
    cc->SetSwkFC(swkFC);
}
#endif // ENGINE_SERIAL_HPP