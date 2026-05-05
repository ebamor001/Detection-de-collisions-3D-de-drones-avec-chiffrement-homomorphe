#ifndef ENGINE_SERIAL_HPP
#define ENGINE_SERIAL_HPP

#include "engine.hpp"
#include "openfhe.h"
#include "cryptocontext-ser.h"
#include "key/key-ser.h"
#include "scheme/ckksrns/ckksrns-ser.h"
#include <sstream>
#include <fstream>

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

#endif // ENGINE_SERIAL_HPP
