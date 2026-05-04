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
// MODE DETECT — Pipeline FHE complet côté Alice (CORRIGÉ)
//
// FAILLE ORIGINALE (lignes 225-226) :
//   cc->Decrypt(sk, ct1, &pt1);  // ← coordonnées en CLAIR en mémoire
//   cc->Decrypt(sk, ct2, &pt2);  // ← coordonnées en CLAIR en mémoire
//   Segment s1 = { IntPoint{v1[0]...} };  // ← passage en clair à geo
//   geo.checkSegmentIntersection3D(s1, s2); // ← "FHE" sur données en clair
//
// CORRECTION :
//   Les coordonnées ne sont JAMAIS déchiffrées.
//   On enchaîne directement :
//     1. Arithmétique CKKS sur les 12 ciphertexts (prod. vectoriel, orientations)
//     2. loadContext() + setupSchemeSwitching() avec sk_Alice
//     3. isNearZeroBand + compareGreaterThanZero (scheme switching réel)
//     4. Déchiffrement du seul bit de résultat final
//
// Interface : même format 6 fichiers que modeDetectEncrypted
//   --ct1 <préfixe_alice>  (_p1x, _p1y, _p1z, _q1x, _q1y, _q1z)
//   --ct2 <préfixe_bob>    (même structure)
// ══════════════════════════════════════════════════════════════════════════════
static int modeDetect(const std::map<std::string,std::string>& args) {
    std::string ctx_file  = args.count("--ctx") ? args.at("--ctx") : "ctx.bin";
    std::string emk_file  = args.count("--emk") ? args.at("--emk") : "emk.bin";
    std::string sk_file   = args.count("--sk")  ? args.at("--sk")  : "sk.bin";
    std::string pk_file   = args.count("--pk")  ? args.at("--pk")  : "pk.bin";
    std::string ct1_pref  = args.count("--ct1") ? args.at("--ct1") : "ct_alice";
    std::string ct2_pref  = args.count("--ct2") ? args.at("--ct2") : "ct_bob";

    if (sk_file.empty()) {
        std::cerr << "[detect] Erreur : --sk requis\n";
        return 1;
    }

    std::cout << "[detect] Chargement contexte...\n"; std::cout.flush();
    auto cc = deserializeContext(readFile(ctx_file));
    deserializeEvalMultKeys(cc, readFile(emk_file));
    auto sk = deserializeSecretKey(readFile(sk_file));
    auto pk = deserializePublicKey(readFile(pk_file));

    // ── Charger les 12 ciphertexts (6 Alice + 6 Bob) ─────────────────────────
    auto loadCt = [&](const std::string& pref, const std::string& s) {
        return deserializeCiphertext(readFile(pref + s + ".bin"));
    };
    auto ct_p1x = loadCt(ct1_pref, "_p1x");
    auto ct_p1y = loadCt(ct1_pref, "_p1y");
    auto ct_p1z = loadCt(ct1_pref, "_p1z");
    auto ct_q1x = loadCt(ct1_pref, "_q1x");
    auto ct_q1y = loadCt(ct1_pref, "_q1y");
    auto ct_q1z = loadCt(ct1_pref, "_q1z");

    auto ct_p2x = loadCt(ct2_pref, "_p1x");
    auto ct_p2y = loadCt(ct2_pref, "_p1y");
    auto ct_p2z = loadCt(ct2_pref, "_p1z");
    auto ct_q2x = loadCt(ct2_pref, "_q1x");
    auto ct_q2y = loadCt(ct2_pref, "_q1y");
    auto ct_q2z = loadCt(ct2_pref, "_q1z");

    std::cout << "[detect] 12 ciphertexts chargés — arithmétique CKKS...\n";
    std::cout.flush();
    auto t0 = std::chrono::high_resolution_clock::now();

    // ── Arithmétique CKKS pure (aucun déchiffrement) ──────────────────────────
    auto ct_d1x = cc->EvalSub(ct_q1x, ct_p1x);
    auto ct_d1y = cc->EvalSub(ct_q1y, ct_p1y);
    auto ct_d1z = cc->EvalSub(ct_q1z, ct_p1z);
    auto ct_d2x = cc->EvalSub(ct_q2x, ct_p2x);
    auto ct_d2y = cc->EvalSub(ct_q2y, ct_p2y);
    auto ct_d2z = cc->EvalSub(ct_q2z, ct_p2z);

    // Produit vectoriel n = d1 × d2
    auto ct_nx = cc->EvalSub(cc->EvalMult(ct_d1y,ct_d2z), cc->EvalMult(ct_d1z,ct_d2y));
    auto ct_ny = cc->EvalSub(cc->EvalMult(ct_d1z,ct_d2x), cc->EvalMult(ct_d1x,ct_d2z));
    auto ct_nz = cc->EvalSub(cc->EvalMult(ct_d1x,ct_d2y), cc->EvalMult(ct_d1y,ct_d2x));

    // Produit mixte → coplanarité
    auto ct_wx  = cc->EvalSub(ct_p2x, ct_p1x);
    auto ct_wy  = cc->EvalSub(ct_p2y, ct_p1y);
    auto ct_wz  = cc->EvalSub(ct_p2z, ct_p1z);
    auto ct_cop = cc->EvalAdd(
        cc->EvalAdd(cc->EvalMult(ct_wx,ct_nx), cc->EvalMult(ct_wy,ct_ny)),
        cc->EvalMult(ct_wz,ct_nz));

    // Orientations 2D
    auto ct_o1 = cc->EvalSub(
        cc->EvalMult(ct_d1y, cc->EvalSub(ct_p2x,ct_q1x)),
        cc->EvalMult(ct_d1x, cc->EvalSub(ct_p2y,ct_q1y)));
    auto ct_o2 = cc->EvalSub(
        cc->EvalMult(ct_d1y, cc->EvalSub(ct_q2x,ct_q1x)),
        cc->EvalMult(ct_d1x, cc->EvalSub(ct_q2y,ct_q1y)));
    auto ct_o3 = cc->EvalSub(
        cc->EvalMult(ct_d2y, cc->EvalSub(ct_p1x,ct_q2x)),
        cc->EvalMult(ct_d2x, cc->EvalSub(ct_p1y,ct_q2y)));
    auto ct_o4 = cc->EvalSub(
        cc->EvalMult(ct_d2y, cc->EvalSub(ct_q1x,ct_q2x)),
        cc->EvalMult(ct_d2x, cc->EvalSub(ct_q1y,ct_q2y)));

    // ── Scheme switching avec sk_Alice ────────────────────────────────────────
    std::cout << "[detect] Scheme switching...\n"; std::cout.flush();
    KeyPair<DCRTPoly> aliceKeys;
    aliceKeys.secretKey = sk;
    aliceKeys.publicKey = pk;
    CryptoEngine engine;
    CryptoEngine::Config config;
    config.batchSize = 64;
    config.switchValues = 64;
    config.logQ_ccLWE = 25;

    engine.loadContext(cc, aliceKeys, config);
    engine.setupSchemeSwitching();

    // Étape 1 : coplanarité (2 SS) — sortie anticipée si non coplanaire
    auto ct_copOK = engine.isNearZeroBand(ct_cop, 1.0);
    double copVal = engine.decryptValue(ct_copOK);
    bool coplanar = (copVal > 0.5);

    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double,std::milli>(t1-t0).count();

    if (!coplanar) {
        std::cout << "[detect] LIBRE — non coplanaire"
                  << " (" << std::fixed << std::setprecision(0) << ms << " ms, 2 SS)\n";
        std::cout << "\nJSON_RESULT:{\"collision\":false"
                  << ",\"scheme_switches\":2"
                  << ",\"total_time_ms\":" << std::fixed << std::setprecision(1) << ms
                  << "}\n";
        std::cout.flush();
        return 0;
    }

    // Étape 2 : orientations (4 SS supplémentaires)
    auto ct_s1    = engine.compareGreaterThanZero(ct_o1);
    auto ct_s2    = engine.compareGreaterThanZero(ct_o2);
    auto ct_s3    = engine.compareGreaterThanZero(ct_o3);
    auto ct_s4    = engine.compareGreaterThanZero(ct_o4);
    auto ct_xor12 = engine.eXor(ct_s1, ct_s2);
    auto ct_xor34 = engine.eXor(ct_s3, ct_s4);
    auto ct_inter = engine.eAnd(ct_xor12, ct_xor34);
    auto ct_final = engine.eAnd(ct_copOK, ct_inter);

    // Seul le bit final est déchiffré — jamais les coordonnées
    double result  = engine.decryptValue(ct_final);
    bool collision = (result > 0.5);

    auto t2 = std::chrono::high_resolution_clock::now();
    ms = std::chrono::duration<double,std::milli>(t2-t0).count();

    std::cout << "[detect] " << (collision ? "COLLISION" : "LIBRE")
              << " (" << std::fixed << std::setprecision(0) << ms << " ms, 6 SS)\n";
    std::cout << "\nJSON_RESULT:{\"collision\":"  << (collision ? "true" : "false")
              << ",\"scheme_switches\":6"
              << ",\"total_time_ms\":"            << std::fixed << std::setprecision(1) << ms
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

// ══════════════════════════════════════════════════════════════════════════════
// MODE DETECT_ENCRYPTED (CORRIGÉ) — Bob côté
//
// CORRECTION DU BUG CRITIQUE :
//   Version originale : un CryptoEngine était réinitialisé ici avec de NOUVELLES
//   clés (engine.initialize()), puis engine.decryptValue() tentait de déchiffrer
//   des ciphertexts chiffrés avec les CLÉS D'ALICE → résultat invalide garanti.
//
// CORRECTION :
//   - Aucun engine n'est créé côté Bob. Toutes les opérations FHE utilisent
//     exclusivement le contexte `cc` chargé depuis Alice (avec ses eval keys).
//   - Bob NE déchiffre JAMAIS. Il écrit les ciphertexts de résultat dans des
//     fichiers, qu'Alice récupère pour les déchiffrer avec SA clé secrète.
//   - Sortie : 5 fichiers ciphertext (cop, o1, o2, o3, o4) + préfixe sur stdout
// ══════════════════════════════════════════════════════════════════════════════
static int modeDetectEncrypted(const std::map<std::string,std::string>& args) {
    std::string ctx_file  = args.count("--ctx")  ? args.at("--ctx")  : "ctx.bin";
    std::string emk_file  = args.count("--emk")  ? args.at("--emk")  : "emk.bin";
    std::string ct1_pref  = args.count("--ct1")  ? args.at("--ct1")  : "ct_alice";
    std::string ct2_pref  = args.count("--ct2")  ? args.at("--ct2")  : "ct_bob";
    std::string out_pref  = args.count("--out")  ? args.at("--out")  : "ct_result";

    auto cc = deserializeContext(readFile(ctx_file));
    deserializeEvalMultKeys(cc, readFile(emk_file));

    auto loadCt = [&](const std::string& pref, const std::string& s) {
        return deserializeCiphertext(readFile(pref + s + ".bin"));
    };

    auto p1x = loadCt(ct1_pref, "_p1x");
    auto p1y = loadCt(ct1_pref, "_p1y");
    auto p1z = loadCt(ct1_pref, "_p1z");
    auto q1x = loadCt(ct1_pref, "_q1x");
    auto q1y = loadCt(ct1_pref, "_q1y");
    auto q1z = loadCt(ct1_pref, "_q1z");

    auto p2x = loadCt(ct2_pref, "_p1x");
    auto p2y = loadCt(ct2_pref, "_p1y");
    auto p2z = loadCt(ct2_pref, "_p1z");
    auto q2x = loadCt(ct2_pref, "_q1x");
    auto q2y = loadCt(ct2_pref, "_q1y");
    auto q2z = loadCt(ct2_pref, "_q1z");

    std::cout << "[detect_enc] 12 ciphertexts charges — calcul CKKS 3D\n";
    std::cout.flush();

    auto t0 = std::chrono::high_resolution_clock::now();

    auto d1x = cc->EvalSub(q1x, p1x);
    auto d1y = cc->EvalSub(q1y, p1y);
    auto d1z = cc->EvalSub(q1z, p1z);

    auto d2x = cc->EvalSub(q2x, p2x);
    auto d2y = cc->EvalSub(q2y, p2y);
    auto d2z = cc->EvalSub(q2z, p2z);

    auto nx = cc->EvalSub(cc->EvalMult(d1y, d2z), cc->EvalMult(d1z, d2y));
    auto ny = cc->EvalSub(cc->EvalMult(d1z, d2x), cc->EvalMult(d1x, d2z));
    auto nz = cc->EvalSub(cc->EvalMult(d1x, d2y), cc->EvalMult(d1y, d2x));

    auto nx2 = cc->EvalMult(nx, nx);
    auto ny2 = cc->EvalMult(ny, ny);
    auto nz2 = cc->EvalMult(nz, nz);

    auto wx = cc->EvalSub(p2x, p1x);
    auto wy = cc->EvalSub(p2y, p1y);
    auto wz = cc->EvalSub(p2z, p1z);

    auto cop = cc->EvalAdd(
        cc->EvalAdd(cc->EvalMult(wx, nx), cc->EvalMult(wy, ny)),
        cc->EvalMult(wz, nz)
    );

    auto orient = [&](const auto& ax, const auto& ay,
                      const auto& bx, const auto& by,
                      const auto& cx, const auto& cy) {
        auto dy1 = cc->EvalSub(by, ay);
        auto dx1 = cc->EvalSub(bx, ax);
        auto dx2 = cc->EvalSub(cx, bx);
        auto dy2 = cc->EvalSub(cy, by);

        return cc->EvalSub(
            cc->EvalMult(dy1, dx2),
            cc->EvalMult(dx1, dy2)
        );
    };

    // Plan XY = DROP_Z
    auto o1_xy = orient(p1x, p1y, q1x, q1y, p2x, p2y);
    auto o2_xy = orient(p1x, p1y, q1x, q1y, q2x, q2y);
    auto o3_xy = orient(p2x, p2y, q2x, q2y, p1x, p1y);
    auto o4_xy = orient(p2x, p2y, q2x, q2y, q1x, q1y);

    // Plan XZ = DROP_Y
    auto o1_xz = orient(p1x, p1z, q1x, q1z, p2x, p2z);
    auto o2_xz = orient(p1x, p1z, q1x, q1z, q2x, q2z);
    auto o3_xz = orient(p2x, p2z, q2x, q2z, p1x, p1z);
    auto o4_xz = orient(p2x, p2z, q2x, q2z, q1x, q1z);

    // Plan YZ = DROP_X
    auto o1_yz = orient(p1y, p1z, q1y, q1z, p2y, p2z);
    auto o2_yz = orient(p1y, p1z, q1y, q1z, q2y, q2z);
    auto o3_yz = orient(p2y, p2z, q2y, q2z, p1y, p1z);
    auto o4_yz = orient(p2y, p2z, q2y, q2z, q1y, q1z);

    auto p12_xy = cc->EvalMult(o1_xy, o2_xy);
    auto p34_xy = cc->EvalMult(o3_xy, o4_xy);

    auto p12_xz = cc->EvalMult(o1_xz, o2_xz);
    auto p34_xz = cc->EvalMult(o3_xz, o4_xz);

    auto p12_yz = cc->EvalMult(o1_yz, o2_yz);
    auto p34_yz = cc->EvalMult(o3_yz, o4_yz);

    writeFile(out_pref + "_cop.bin",    serializeCiphertext(cop));

    writeFile(out_pref + "_nx2.bin",    serializeCiphertext(nx2));
    writeFile(out_pref + "_ny2.bin",    serializeCiphertext(ny2));
    writeFile(out_pref + "_nz2.bin",    serializeCiphertext(nz2));

    writeFile(out_pref + "_p12_xy.bin", serializeCiphertext(p12_xy));
    writeFile(out_pref + "_p34_xy.bin", serializeCiphertext(p34_xy));

    writeFile(out_pref + "_p12_xz.bin", serializeCiphertext(p12_xz));
    writeFile(out_pref + "_p34_xz.bin", serializeCiphertext(p34_xz));

    writeFile(out_pref + "_p12_yz.bin", serializeCiphertext(p12_yz));
    writeFile(out_pref + "_p34_yz.bin", serializeCiphertext(p34_yz));

    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double,std::milli>(t1 - t0).count();

    std::cout << "[detect_enc] 10 ciphertexts 3D ecrits avec drop axis implicite ("
              << std::fixed << std::setprecision(0) << ms << " ms)\n";
    std::cout << "CT_RESULT_PREFIX:" << out_pref << "\n";
    std::cout.flush();

    return 0;
}
// ══════════════════════════════════════════════════════════════════════════════
// MODE DECRYPT_RESULT (NOUVEAU) — Alice côté
//
// Alice reçoit les 5 ciphertexts calculés par Bob, les déchiffre avec SA clé
// ══════════════════════════════════════════════════════════════════════════════
// MODE DECRYPT_WITH_SS — Alice côté, avec VRAI scheme switching
//
// Bob a calculé ct_cop, ct_o1..o4 (arithmétique CKKS pure, sans SS).
// Alice :
//   1. Charge son propre contexte CKKS + clés (pk, sk, emk)
//   2. Injecte ce contexte dans un CryptoEngine via loadContext()
//      → PAS de engine.initialize() qui créerait de nouvelles clés
//   3. Appelle setupSchemeSwitching() avec sk_Alice → clés SS cohérentes
//   4. Applique isNearZeroBand + compareGreaterThanZero (6 SS au total)
//   5. Combine en booléen chiffré (eXor / eAnd) et déchiffre le bit final
//
// Pourquoi ça marche :
//   Les ciphertexts de Bob sont CKKS valides sous pk_Alice.
//   Le scheme switching utilise sk_Alice → extrait correctement les signes.
//   La sk ne quitte jamais Alice.
// ══════════════════════════════════════════════════════════════════════════════
static int modeDecryptWithSS(const std::map<std::string,std::string>& args) {
    std::string ctx_file = args.count("--ctx") ? args.at("--ctx") : "ctx.bin";
    std::string sk_file  = args.count("--sk")  ? args.at("--sk")  : "sk.bin";
    std::string pk_file  = args.count("--pk")  ? args.at("--pk")  : "pk.bin";
    std::string emk_file = args.count("--emk") ? args.at("--emk") : "emk.bin";
    std::string ct_pref  = args.count("--ct")  ? args.at("--ct")  : "ct_result";

    auto cc = deserializeContext(readFile(ctx_file));
    deserializeEvalMultKeys(cc, readFile(emk_file));
    auto sk = deserializeSecretKey(readFile(sk_file));
    auto pk = deserializePublicKey(readFile(pk_file));

    KeyPair<DCRTPoly> aliceKeys;
    aliceKeys.secretKey = sk;
    aliceKeys.publicKey = pk;

    CryptoEngine engine;
    CryptoEngine::Config config;
    config.batchSize = 64;
    config.switchValues = 64;
    config.logQ_ccLWE = 25;

    engine.loadContext(cc, aliceKeys, config);
    engine.setupSchemeSwitching();

    auto loadCt = [&](const std::string& s) {
        return deserializeCiphertext(readFile(ct_pref + s + ".bin"));
    };

    auto ct_cop = loadCt("_cop");

    auto nx2 = loadCt("_nx2");
    auto ny2 = loadCt("_ny2");
    auto nz2 = loadCt("_nz2");

    auto p12_xy = loadCt("_p12_xy");
    auto p34_xy = loadCt("_p34_xy");

    auto p12_xz = loadCt("_p12_xz");
    auto p34_xz = loadCt("_p34_xz");

    auto p12_yz = loadCt("_p12_yz");
    auto p34_yz = loadCt("_p34_yz");

    std::cout << "[decrypt_ss] Choix drop axis + comparaisons...\n";
    std::cout.flush();

    auto t0 = std::chrono::high_resolution_clock::now();

    // 1) Coplanarité : 2 SS
    auto ct_copOK = engine.isNearZeroBand(ct_cop, 1.0);
    double copVal = engine.decryptValue(ct_copOK);
    bool coplanar = (copVal > 0.5);

    if (!coplanar) {
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double,std::milli>(t1 - t0).count();

        std::cout << "[decrypt_ss] LIBRE — non coplanaire"
                  << " (" << std::fixed << std::setprecision(0) << ms << " ms, 2 SS)\n";
        std::cout << "JSON_RESULT:{\"collision\":false"
                  << ",\"scheme_switches\":2"
                  << ",\"total_time_ms\":" << std::fixed << std::setprecision(1) << ms
                  << "}\n";
        return 0;
    }

    // 2) Choix du drop axis : 2 SS
    // bxy = 1 si nx² > ny²
    auto bxy = engine.compareGreaterThanZero(cc->EvalSub(nx2, ny2));
    auto one = engine.oneLike(bxy);
    auto not_bxy = engine.eNot(bxy);

    auto max_xy = cc->EvalAdd(
        cc->EvalMult(bxy, nx2),
        cc->EvalMult(not_bxy, ny2)
    );

    // bxyz = 1 si max(nx²,ny²) > nz²
    auto bxyz = engine.compareGreaterThanZero(cc->EvalSub(max_xy, nz2));
    auto not_bxyz = engine.eNot(bxyz);

    // chooseX = DROP_X => plan YZ
    // chooseY = DROP_Y => plan XZ
    // chooseZ = DROP_Z => plan XY
    auto chooseX = cc->EvalMult(bxyz, bxy);
    auto chooseY = cc->EvalMult(bxyz, not_bxy);
    auto chooseZ = not_bxyz;

    // 3) Sélection chiffrée du bon p12/p34
    auto p12 = cc->EvalAdd(
        cc->EvalAdd(
            cc->EvalMult(chooseX, p12_yz),
            cc->EvalMult(chooseY, p12_xz)
        ),
        cc->EvalMult(chooseZ, p12_xy)
    );

    auto p34 = cc->EvalAdd(
        cc->EvalAdd(
            cc->EvalMult(chooseX, p34_yz),
            cc->EvalMult(chooseY, p34_xz)
        ),
        cc->EvalMult(chooseZ, p34_xy)
    );

    // 4) Intersection 2D du bon plan : 2 SS
    auto opp12 = engine.ltZero(p12);
    auto opp34 = engine.ltZero(p34);

    auto inter2D = engine.eAnd(opp12, opp34);
    auto finalCt = engine.eAnd(ct_copOK, inter2D);

    double result = engine.decryptValue(finalCt);
    bool collision = (result > 0.5);

    auto t2 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double,std::milli>(t2 - t0).count();

    std::cout << "[decrypt_ss] " << (collision ? "COLLISION" : "LIBRE")
              << " (" << std::fixed << std::setprecision(0) << ms << " ms, 6 SS)\n";

    std::cout << "JSON_RESULT:{\"collision\":" << (collision ? "true" : "false")
              << ",\"scheme_switches\":6"
              << ",\"total_time_ms\":" << std::fixed << std::setprecision(1) << ms
              << "}\n";

    return 0;
}

int main(int argc, char* argv[]) {
    auto args = parseArgs(argc, argv);
    std::string mode = args.count("--mode") ? args.at("--mode") : "demo";

    try {
        if      (mode == "init")             return modeInit(args);
        else if (mode == "keygen")           return modeKeygen(args);
        else if (mode == "encrypt")          return modeEncrypt(args);
        else if (mode == "detect")           return modeDetect(args);
        else if (mode == "detect_encrypted") return modeDetectEncrypted(args);
        else if (mode == "decrypt_with_ss")  return modeDecryptWithSS(args);
        else                                 return modeDemo(args);
    } catch (const std::exception& e) {
        std::cerr << "Erreur : " << e.what() << "\n";
        return 1;
    }
}