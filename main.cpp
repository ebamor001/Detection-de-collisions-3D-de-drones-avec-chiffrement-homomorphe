/**
 * main.cpp — Detection collision 3D FHE
 *
 * Mode demo (existant) :
 *   ./drone_fhe --path1 f1.txt --path2 f2.txt [--path3 f3.txt]
 *
 * Modes communication Alice/Bob :
 *   ./drone_fhe --mode init    --ctx ctx.bin --pk pk.bin --emk emk.bin --esk esk.bin
 *   ./drone_fhe --mode keygen  --ctx ctx.bin --pk pk_out.bin
 *   ./drone_fhe --mode encrypt --ctx ctx.bin --pk pk.bin --path traj.txt --out ct.bin
 *   ./drone_fhe --mode detect  --ctx ctx.bin --ct1 ct1.bin --ct2 ct2.bin
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <iomanip>

#include "geometry.hpp"
#include "engine.hpp"
#include "engine_serial.hpp"
#include "types.hpp"
#include "io.hpp"
#include "intersection.hpp"

// ── Helpers ───────────────────────────────────────────────────────────────────

static std::map<std::string, std::string> parseArgs(int argc, char* argv[]) {
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc - 1; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) == 0) { args[key] = argv[i+1]; ++i; }
    }
    // flag sans valeur
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) == 0 && !args.count(key))
            args[key] = "1";
    }
    return args;
}

static void writeFile(const std::string& path, const std::string& data) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Impossible d'ecrire : " + path);
    f.write(data.data(), data.size());
}

static std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Impossible de lire : " + path);
    return std::string(std::istreambuf_iterator<char>(f), {});
}

// ══════════════════════════════════════════════════════════════════════════════
// MODE INIT — Alice genere le contexte et les cles, les sauvegarde sur disque
// Appele par alice.py au demarrage
// ══════════════════════════════════════════════════════════════════════════════

static int modeInit(const std::map<std::string,std::string>& args) {
    std::string ctx_file = args.count("--ctx") ? args.at("--ctx") : "ctx.bin";
    std::string pk_file  = args.count("--pk")  ? args.at("--pk")  : "pk.bin";
    std::string emk_file = args.count("--emk") ? args.at("--emk") : "emk.bin";
    std::string esk_file = args.count("--esk") ? args.at("--esk") : "esk.bin";
    std::string sk_file  = args.count("--sk")  ? args.at("--sk")  : "sk.bin";

    std::cout << "[init] Generation du contexte FHE...\n"; std::cout.flush();

    CryptoEngine engine;
    engine.initialize();
    engine.setupSchemeSwitching();

    auto cc = engine.getCKKSContext();

    // Serialiser le contexte + toutes les cles
    std::cout << "[init] Serialisation...\n"; std::cout.flush();
    writeFile(ctx_file,  serializeContext(cc));
    writeFile(pk_file,   serializePublicKey(engine.getKeys().publicKey));
    writeFile(emk_file,  serializeEvalMultKeys(cc));
    writeFile(esk_file,  serializeEvalSumKeys(cc));
    writeFile(sk_file,   serializeSecretKey(engine.getKeys().secretKey));

    std::cout << "[init] OK\n";
    std::cout << "  ctx  : " << ctx_file  << "\n";
    std::cout << "  pk   : " << pk_file   << "\n";
    std::cout << "  sk   : " << sk_file   << "\n";
    std::cout << "  emk  : " << emk_file  << "\n";
    std::cout << "  esk  : " << esk_file  << "\n";
    return 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// MODE KEYGEN — Bob charge le contexte d'Alice et genere ses propres cles
// Appele par bob.py apres reception du contexte
// ══════════════════════════════════════════════════════════════════════════════

static int modeKeygen(const std::map<std::string,std::string>& args) {
    std::string ctx_file = args.count("--ctx") ? args.at("--ctx") : "ctx.bin";
    std::string pk_file  = args.count("--pk")  ? args.at("--pk")  : "pk_bob.bin";

    std::cout << "[keygen] Chargement du contexte...\n"; std::cout.flush();

    // Charger le contexte d'Alice
    auto cc = deserializeContext(readFile(ctx_file));

    // Generer une nouvelle paire de cles pour Bob
    std::cout << "[keygen] Generation des cles de Bob...\n"; std::cout.flush();
    auto bobKeys = cc->KeyGen();
    cc->EvalMultKeyGen(bobKeys.secretKey);

    // Rotation keys
    std::vector<int32_t> rotIndices;
    for (int32_t i = 1; i < 2048; i *= 2) {
        rotIndices.push_back(i);
        rotIndices.push_back(-i);
    }
    cc->EvalRotateKeyGen(bobKeys.secretKey, rotIndices);

    // Sauvegarder la cle publique de Bob
    writeFile(pk_file, serializePublicKey(bobKeys.publicKey));

    std::cout << "[keygen] OK — cle publique : " << pk_file << "\n";
    return 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// MODE ENCRYPT — Chiffre une trajectoire avec la cle publique du destinataire
// Appele par alice.py ET bob.py
// ══════════════════════════════════════════════════════════════════════════════

static int modeEncrypt(const std::map<std::string,std::string>& args) {
    std::string ctx_file  = args.count("--ctx")  ? args.at("--ctx")  : "ctx.bin";
    std::string pk_file   = args.count("--pk")   ? args.at("--pk")   : "pk.bin";
    std::string path_file = args.count("--path") ? args.at("--path") : "";
    std::string out_file  = args.count("--out")  ? args.at("--out")  : "ct.bin";

    if (path_file.empty()) {
        std::cerr << "Usage: --mode encrypt --ctx ctx.bin --pk pk.bin --path traj.txt --out ct.bin\n";
        return 1;
    }

    std::cout << "[encrypt] Chargement du contexte...\n"; std::cout.flush();
    auto cc = deserializeContext(readFile(ctx_file));
    auto pk = deserializePublicKey(readFile(pk_file));

    // Charger la trajectoire
    Path path = PathIO::read_single_path(path_file);
    std::cout << "[encrypt] Trajectoire : " << path.size() << " points\n";

    // Chiffrer le premier segment (pour la demo Alice/Bob)
    // En production on chiffrerait tous les segments
    if (path.size() < 2) {
        std::cerr << "Trajectoire trop courte (minimum 2 points)\n";
        return 1;
    }

    // Chiffrer chaque coordonnee dans un ciphertext separe (slot 0)
    // Format : 6 fichiers ct_p1x.bin, ct_p1y.bin, ct_p1z.bin, ct_q1x.bin, ct_q1y.bin, ct_q1z.bin
    // Chaque ciphertext contient UNE seule coordonnee en slot 0
    // => compareGreaterThanZero fonctionne directement sur slot 0

    std::vector<std::string> suffixes = {"_p1x","_p1y","_p1z","_q1x","_q1y","_q1z"};
    std::vector<double> values = {
        (double)path[0].x, (double)path[0].y, (double)path[0].z,
        (double)path[1].x, (double)path[1].y, (double)path[1].z
    };

    for (int i = 0; i < 6; i++) {
        std::vector<double> v(64, 0.0);
        v[0] = values[i];
        auto pt_i = cc->MakeCKKSPackedPlaintext(v);
        auto ct_i = cc->Encrypt(pk, pt_i);
        std::string fname = out_file + suffixes[i] + ".bin";
        writeFile(fname, serializeCiphertext(ct_i));
    }

    std::cout << "[encrypt] OK — 6 ciphertexts generes avec prefixe : " << out_file << "\n";
    return 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// MODE DETECT — Calcul de collision FHE sur les deux ciphertexts
// Appele par alice.py apres reception du ciphertext de Bob
// ══════════════════════════════════════════════════════════════════════════════

static int modeDetect(const std::map<std::string,std::string>& args) {
    std::string ctx_file  = args.count("--ctx")  ? args.at("--ctx")  : "ctx.bin";
    std::string emk_file  = args.count("--emk")  ? args.at("--emk")  : "emk.bin";
    std::string esk_file  = args.count("--esk")  ? args.at("--esk")  : "esk.bin";
    std::string ct1_file  = args.count("--ct1")  ? args.at("--ct1")  : "ct1.bin";
    std::string ct2_file  = args.count("--ct2")  ? args.at("--ct2")  : "ct2.bin";
    std::string sk_file   = args.count("--sk")   ? args.at("--sk")   : "";

    std::cout << "[detect] Chargement du contexte et des cles...\n"; std::cout.flush();

    auto cc = deserializeContext(readFile(ctx_file));
    deserializeEvalMultKeys(cc, readFile(emk_file));
    deserializeEvalSumKeys(cc,  readFile(esk_file));

    // Charger les ciphertexts
    auto ct1 = deserializeCiphertext(readFile(ct1_file));
    auto ct2 = deserializeCiphertext(readFile(ct2_file));

    std::cout << "[detect] Ciphertexts charges\n"; std::cout.flush();

    // Charger la cle secrete d'Alice depuis le fichier
    if (sk_file.empty()) {
        std::cerr << "Erreur : --sk requis pour le mode detect\n";
        return 1;
    }
    auto sk = deserializeSecretKey(readFile(sk_file));

    // Initialiser engine et geo avec le contexte charge
    CryptoEngine engine;
    engine.initialize();
    engine.setupSchemeSwitching();
    GeometryEngine geo(&engine);

    // Decoder les coordonnees depuis les ciphertexts
    // avec la VRAIE cle secrete d'Alice
    Plaintext pt1, pt2;
    cc->Decrypt(sk, ct1, &pt1);
    cc->Decrypt(sk, ct2, &pt2);

    auto t0 = std::chrono::high_resolution_clock::now();

    auto v1 = pt1->GetCKKSPackedValue();
    auto v2 = pt2->GetCKKSPackedValue();

    // Reconstruire les segments
    Segment s1 = {
        IntPoint{(long)v1[0].real(), (long)v1[1].real(), (long)v1[2].real()},
        IntPoint{(long)v1[3].real(), (long)v1[4].real(), (long)v1[5].real()}
    };
    Segment s2 = {
        IntPoint{(long)v2[0].real(), (long)v2[1].real(), (long)v2[2].real()},
        IntPoint{(long)v2[3].real(), (long)v2[4].real(), (long)v2[5].real()}
    };

    std::cout << "[detect] Calcul FHE en cours...\n"; std::cout.flush();

    // Detection FHE
    auto ct_result = geo.checkSegmentIntersection3D(s1, s2);
    double result  = engine.decryptValue(ct_result);
    bool collision = (result > 0.5);

    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double,std::milli>(t1-t0).count();

    std::cout << "[detect] Resultat : " << (collision ? "COLLISION" : "LIBRE") << "\n";
    std::cout << "[detect] Temps    : " << std::fixed << std::setprecision(0) << ms << " ms\n";

    // JSON pour alice.py
    std::cout << "\nJSON_RESULT:{"
              << "\"collision\":"       << (collision ? "true" : "false") << ","
              << "\"scheme_switches\":" << geo.getBootstrapCount() << ","
              << "\"total_time_ms\":"   << std::fixed << std::setprecision(1) << ms
              << "}\n";
    std::cout.flush();
    return 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// MODE DEMO (existant) — trajectoires depuis fichiers, batching
// ══════════════════════════════════════════════════════════════════════════════

static int modeDemo(const std::map<std::string,std::string>& args) {
    std::cout << "=== Drone FHE 3D - Batching ===" << std::endl;
    std::cout.flush();

    std::string f1 = args.count("--path1") ? args.at("--path1") : "";
    std::string f2 = args.count("--path2") ? args.at("--path2") : "";
    std::string f3 = args.count("--path3") ? args.at("--path3") : "";

    if (f1.empty() || f2.empty()) {
        std::cerr << "Usage: ./drone_fhe --path1 f1.txt --path2 f2.txt [--path3 f3.txt]\n";
        return 1;
    }

    std::vector<Path>        paths;
    std::vector<std::string> names;
    try {
        paths.push_back(PathIO::read_single_path(f1)); names.push_back("Drone 1");
        paths.push_back(PathIO::read_single_path(f2)); names.push_back("Drone 2");
        if (!f3.empty()) {
            paths.push_back(PathIO::read_single_path(f3)); names.push_back("Drone 3");
        }
        for (size_t i = 0; i < paths.size(); ++i)
            std::cout << "  " << names[i] << " : " << paths[i].size() << " points\n";
    } catch (const std::exception& e) {
        std::cerr << "Erreur lecture : " << e.what() << "\n";
        return 1;
    }

    auto t0 = std::chrono::high_resolution_clock::now();

    CryptoEngine engine;
    engine.initialize();
    engine.setupSchemeSwitching();
    GeometryEngine geo(&engine);

    auto t_ready = std::chrono::high_resolution_clock::now();
    double ms_setup = std::chrono::duration<double,std::milli>(t_ready - t0).count();
    std::cout << "  Setup : " << std::fixed << std::setprecision(1)
              << ms_setup/1000 << " s\n";
    std::cout.flush();

    struct PairResult { std::string name; bool collision; double timeMs; };
    std::vector<PairResult> pairResults;
    int totalCollisions = 0;

    for (size_t i = 0; i < paths.size(); ++i) {
        for (size_t j = i + 1; j < paths.size(); ++j) {
            std::string pairName = names[i] + " x " + names[j];
            std::cout << "\n  -- " << pairName << " --\n";

            IntersectionDetector detector(&engine, &geo);
            auto clearRes = detector.detectIntersectionsClear(paths[i], paths[j]);
            int clearHits = 0;
            for (auto& r : clearRes) if (r.intersects) ++clearHits;
            std::cout << "    Clair  : " << clearHits << " collision(s)\n";

            Segment mySeg = {paths[i][0], paths[i][1]};
            std::vector<Segment> neighbors;
            for (size_t k = 0; k+1 < paths[j].size(); ++k)
                neighbors.push_back({paths[j][k], paths[j][k+1]});

            auto tb0 = std::chrono::high_resolution_clock::now();
            auto batchRes = geo.batchCheckIntersection3D(mySeg, neighbors);
            auto tb1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double,std::milli>(tb1-tb0).count();

            bool fheCollision = false;
            for (auto v : batchRes) if (v > 0.5) { fheCollision = true; break; }

            std::cout << "    FHE    : " << (fheCollision ? "COLLISION" : "LIBRE")
                      << "  (" << std::fixed << std::setprecision(0) << ms << " ms)\n";

            pairResults.push_back({pairName, (clearHits > 0), ms});
            if (clearHits > 0) ++totalCollisions;
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double ms_total = std::chrono::duration<double,std::milli>(t_end-t0).count();

    geo.printStats();

    std::cout << "\nJSON_RESULT:{"
              << "\"collision\":"       << (totalCollisions > 0 ? "true" : "false") << ","
              << "\"collisions_count\":" << totalCollisions << ","
              << "\"pairs_tested\":"    << pairResults.size() << ","
              << "\"drones\":"          << paths.size() << ","
              << "\"scheme_switches\":" << geo.getBootstrapCount() << ","
              << "\"total_time_ms\":"   << std::fixed << std::setprecision(1) << ms_total << ","
              << "\"details\":[";

    for (size_t k = 0; k < pairResults.size(); ++k) {
        if (k > 0) std::cout << ",";
        std::cout << "{\"pair\":\"" << pairResults[k].name << "\","
                  << "\"collision\":" << (pairResults[k].collision ? "true" : "false") << ","
                  << "\"time_ms\":" << pairResults[k].timeMs << "}";
    }
    std::cout << "]}\n";
    std::cout.flush();
    return 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// MAIN
// ══════════════════════════════════════════════════════════════════════════════

// ── Mode detect_encrypted ─────────────────────────────────────────────────────
// Bob appelle ce mode avec 12 ciphertexts (6 d'Alice + 6 de Bob)
// Chaque ciphertext contient UNE coordonnee en slot 0
// Calcul FHE complet : produit vectoriel, produit mixte, orientations
static int modeDetectEncrypted(const std::map<std::string,std::string>& args) {
    std::string ctx_file  = args.count("--ctx")  ? args.at("--ctx")  : "ctx.bin";
    std::string emk_file  = args.count("--emk")  ? args.at("--emk")  : "emk.bin";
    std::string esk_file  = args.count("--esk")  ? args.at("--esk")  : "esk.bin";
    std::string ct1_pref  = args.count("--ct1")  ? args.at("--ct1")  : "ct_alice";
    std::string ct2_pref  = args.count("--ct2")  ? args.at("--ct2")  : "ct_bob";

    auto cc = deserializeContext(readFile(ctx_file));
    deserializeEvalMultKeys(cc, readFile(emk_file));
    deserializeEvalSumKeys(cc,  readFile(esk_file));

    // Charger les 6 ciphertexts d'Alice et 6 de Bob
    std::vector<std::string> sfx = {"_p1x","_p1y","_p1z","_q1x","_q1y","_q1z"};

    auto loadCt = [&](const std::string& pref, const std::string& s) {
        return deserializeCiphertext(readFile(pref + s + ".bin"));
    };

    // Alice : P1=(p1x,p1y,p1z), Q1=(q1x,q1y,q1z)
    auto ct_p1x = loadCt(ct1_pref, "_p1x");
    auto ct_p1y = loadCt(ct1_pref, "_p1y");
    auto ct_p1z = loadCt(ct1_pref, "_p1z");
    auto ct_q1x = loadCt(ct1_pref, "_q1x");
    auto ct_q1y = loadCt(ct1_pref, "_q1y");
    auto ct_q1z = loadCt(ct1_pref, "_q1z");

    // Bob : P2=(p2x,p2y,p2z), Q2=(q2x,q2y,q2z)
    auto ct_p2x = loadCt(ct2_pref, "_p1x");
    auto ct_p2y = loadCt(ct2_pref, "_p1y");
    auto ct_p2z = loadCt(ct2_pref, "_p1z");
    auto ct_q2x = loadCt(ct2_pref, "_q1x");
    auto ct_q2y = loadCt(ct2_pref, "_q1y");
    auto ct_q2z = loadCt(ct2_pref, "_q1z");

    // Initialiser engine
    CryptoEngine engine;
    CryptoEngine::Config config;
    config.batchSize    = 64;
    config.switchValues = 64;
    config.logQ_ccLWE   = 25;
    engine.initialize(config);
    engine.setupSchemeSwitching();

    auto t0 = std::chrono::high_resolution_clock::now();

    // ── Direction des segments sous FHE ──────────────────────────────────────
    auto ct_d1x = cc->EvalSub(ct_q1x, ct_p1x);
    auto ct_d1y = cc->EvalSub(ct_q1y, ct_p1y);
    auto ct_d1z = cc->EvalSub(ct_q1z, ct_p1z);

    auto ct_d2x = cc->EvalSub(ct_q2x, ct_p2x);
    auto ct_d2y = cc->EvalSub(ct_q2y, ct_p2y);
    auto ct_d2z = cc->EvalSub(ct_q2z, ct_p2z);

    // ── Produit vectoriel n = d1 x d2 sous FHE ───────────────────────────────
    auto ct_nx = cc->EvalSub(cc->EvalMult(ct_d1y,ct_d2z), cc->EvalMult(ct_d1z,ct_d2y));
    auto ct_ny = cc->EvalSub(cc->EvalMult(ct_d1z,ct_d2x), cc->EvalMult(ct_d1x,ct_d2z));
    auto ct_nz = cc->EvalSub(cc->EvalMult(ct_d1x,ct_d2y), cc->EvalMult(ct_d1y,ct_d2x));

    // ── Produit mixte w = (P2-P1).n sous FHE ─────────────────────────────────
    auto ct_wx = cc->EvalSub(ct_p2x, ct_p1x);
    auto ct_wy = cc->EvalSub(ct_p2y, ct_p1y);
    auto ct_wz = cc->EvalSub(ct_p2z, ct_p1z);

    auto ct_cop = cc->EvalAdd(
        cc->EvalAdd(cc->EvalMult(ct_wx,ct_nx), cc->EvalMult(ct_wy,ct_ny)),
        cc->EvalMult(ct_wz,ct_nz));

    // ── Test coplanarite FHE (2 SS) ───────────────────────────────────────────
    GeometryEngine geo(&engine);
    auto ct_copOK = engine.isNearZeroBand(ct_cop, 1.0);
    size_t ss = 2;

    double copVal = engine.decryptValue(ct_copOK);
    if (copVal < 0.5) {
        std::cout << "JSON_RESULT:{\"collision\":false,\"scheme_switches\":2,"
                  << "\"total_time_ms\":" << std::chrono::duration<double,std::milli>(
                      std::chrono::high_resolution_clock::now()-t0).count()
                  << "}\n";
        return 0;
    }

    // ── Orientations 2D sous FHE sur plan XY (4 SS) ───────────────────────────
    // o1 = d1y*(p2x-q1x) - d1x*(p2y-q1y)
    auto ct_o1 = cc->EvalSub(
        cc->EvalMult(ct_d1y, cc->EvalSub(ct_p2x,ct_q1x)),
        cc->EvalMult(ct_d1x, cc->EvalSub(ct_p2y,ct_q1y)));

    // o2 = d1y*(q2x-q1x) - d1x*(q2y-q1y)
    auto ct_o2 = cc->EvalSub(
        cc->EvalMult(ct_d1y, cc->EvalSub(ct_q2x,ct_q1x)),
        cc->EvalMult(ct_d1x, cc->EvalSub(ct_q2y,ct_q1y)));

    // o3 = d2y*(p1x-q2x) - d2x*(p1y-q2y)
    auto ct_o3 = cc->EvalSub(
        cc->EvalMult(ct_d2y, cc->EvalSub(ct_p1x,ct_q2x)),
        cc->EvalMult(ct_d2x, cc->EvalSub(ct_p1y,ct_q2y)));

    // o4 = d2y*(q1x-q2x) - d2x*(q1y-q2y)
    auto ct_o4 = cc->EvalSub(
        cc->EvalMult(ct_d2y, cc->EvalSub(ct_q1x,ct_q2x)),
        cc->EvalMult(ct_d2x, cc->EvalSub(ct_q1y,ct_q2y)));

    auto ct_s1 = engine.compareGreaterThanZero(ct_o1);
    auto ct_s2 = engine.compareGreaterThanZero(ct_o2);
    auto ct_s3 = engine.compareGreaterThanZero(ct_o3);
    auto ct_s4 = engine.compareGreaterThanZero(ct_o4);
    ss += 4;

    // XOR(s1,s2) AND XOR(s3,s4)
    auto ct_xor12 = engine.eXor(ct_s1, ct_s2);
    auto ct_xor34 = engine.eXor(ct_s3, ct_s4);
    auto ct_inter = engine.eAnd(ct_xor12, ct_xor34);

    double result  = engine.decryptValue(ct_inter);
    bool collision = (result > 0.5);

    auto t1  = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double,std::milli>(t1-t0).count();

    std::cout << "[detect_enc] " << (collision ? "COLLISION" : "LIBRE")
              << " (" << ms << " ms, " << ss << " SS)\n";
    std::cout << "JSON_RESULT:{\"collision\":"  << (collision?"true":"false")
              << ",\"scheme_switches\":"        << ss
              << ",\"total_time_ms\":"          << std::fixed << std::setprecision(1) << ms
              << "}\n";
    std::cout.flush();
    return 0;
}

int main(int argc, char* argv[]) {
    auto args = parseArgs(argc, argv);
    std::string mode = args.count("--mode") ? args.at("--mode") : "demo";

    try {
        if      (mode == "init")    return modeInit(args);
        else if (mode == "keygen")  return modeKeygen(args);
        else if (mode == "encrypt") return modeEncrypt(args);
        else if (mode == "detect")           return modeDetect(args);
        else if (mode == "detect_encrypted") return modeDetectEncrypted(args);
        else                                 return modeDemo(args);
    } catch (const std::exception& e) {
        std::cerr << "Erreur : " << e.what() << "\n";
        return 1;
    }
}
