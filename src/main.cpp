/**
 * main.cpp — Detection collision 3D FHE
 *
 * Mode demo (existant) :
 *   ./drone_fhe --path1 f1.txt --path2 f2.txt [--path3 f3.txt]
 *
 * Modes communication Alice/Bob :
 *   ./drone_fhe --mode init    --ctx ctx.bin
 *   ./drone_fhe --mode encrypt --ctx ctx.bin --pk pk.bin --path traj.txt --out ct.bin
 *   ./drone_fhe --mode detect  --ctx ctx.bin --emk emk.bin --esk esk.bin --ct1 ct1.bin --ct2 ct2.bin --out ct_result.bin
 *   ./drone_fhe --mode decrypt --ctx ctx.bin --sk sk_bob.bin --ct ct_result.bin
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
#include <sstream>
#include <regex>
// ── Helpers ───────────────────────────────────────────────────────────────────

static std::map<std::string, std::string> parseArgs(int argc, char* argv[]) {
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc - 1; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) == 0) { args[key] = argv[i+1]; ++i; }
    }
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
static std::vector<std::string> splitWS(const std::string& s) {
    std::istringstream iss(s);
    std::vector<std::string> v;
    std::string x;
    while (iss >> x) v.push_back(x);
    return v;
}

static Segment parseSegmentFileSimple(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Impossible de lire : " + path);

    std::string line;
    std::getline(in, line);

    std::regex rgx(R"(\((-?\d+),(-?\d+),(-?\d+)\))");
    std::sregex_iterator it(line.begin(), line.end(), rgx);
    std::sregex_iterator end;

    std::vector<IntPoint> pts;
    for (; it != end; ++it) {
        pts.push_back(IntPoint{
            std::stol((*it)[1].str()),
            std::stol((*it)[2].str()),
            std::stol((*it)[3].str())
        });
    }

    if (pts.size() != 2) {
        throw std::runtime_error("Le fichier segment doit contenir exactement 2 points");
    }

    return {pts[0], pts[1]};
}
// ══════════════════════════════════════════════════════════════════════════════
// MODE INIT — Alice genere seulement le contexte partage
// ══════════════════════════════════════════════════════════════════════════════

static int modeInit(const std::map<std::string,std::string>& args) {
    std::string ctx_file = args.count("--ctx") ? args.at("--ctx") : "ctx.bin";
    std::string pk_file  = args.count("--pk")  ? args.at("--pk")  : "pk.bin";
    std::string sk_file  = args.count("--sk")  ? args.at("--sk")  : "sk.bin";
    std::string emk_file = args.count("--emk") ? args.at("--emk") : "emk.bin";
    std::string esk_file = args.count("--esk") ? args.at("--esk") : "esk.bin";
    std::string btk_file   = args.count("--btk")   ? args.at("--btk")   : "btk.bin";
    std::string swkfc_file = args.count("--swkfc") ? args.at("--swkfc") : "swkfc.bin";    std::cout.flush();

    CryptoEngine engine;
    CryptoEngine::Config cfg;
    cfg.batchSize = 8;
    cfg.switchValues = 1;
    cfg.multDepth = 30;
    engine.initialize(cfg);
    engine.setupSchemeSwitching();

    auto cc = engine.getCKKSContext();

    std::cout << "[init] Serialisation...\n";
    std::cout.flush();


    writeFile(ctx_file, serializeContext(cc));
    writeFile(pk_file,  serializePublicKey(engine.getKeys().publicKey));
    writeFile(sk_file,  serializeSecretKey(engine.getKeys().secretKey));
    writeFile(emk_file, serializeEvalMultKeys(cc));
    writeFile(esk_file, serializeEvalSumKeys(cc));
    writeFile(btk_file, serializeBTKey(cc));
    writeFile(swkfc_file, serializeSwkFC(cc));
    std::cout << "[init] OK\n";
    std::cout << "  ctx : " << ctx_file << "\n";
    std::cout << "  pk  : " << pk_file  << "\n";
    std::cout << "  sk  : " << sk_file  << "\n";
    std::cout << "  emk : " << emk_file << "\n";
    std::cout << "  esk : " << esk_file << "\n";
    std::cout << "  btk : " << btk_file << "\n";
    std::cout << "  swkfc : " << swkfc_file << "\n";
    return 0;
}
// ══════════════════════════════════════════════════════════════════════════════
// MODE ENCRYPT — Chiffre une trajectoire avec la cle publique du destinataire
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

    Path path = PathIO::read_single_path(path_file);
    std::cout << "[encrypt] Trajectoire : " << path.size() << " points\n";

    if (path.size() < 2) {
        std::cerr << "Trajectoire trop courte (minimum 2 points)\n";
        return 1;
    }

    std::vector<double> coords = {
        (double)path[0].x, (double)path[0].y, (double)path[0].z,
        (double)path[1].x, (double)path[1].y, (double)path[1].z,
    };

    auto pt = cc->MakeCKKSPackedPlaintext(coords);
    auto ct = cc->Encrypt(pk, pt);

    writeFile(out_file, serializeCiphertext(ct));
    std::cout << "[encrypt] OK — ciphertext : " << out_file << "\n";
    return 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// MODE DETECT — Calcul FHE sur 2 ciphertexts, sans decryption ici
// Le resultat chiffre est renvoye a Bob
// ══════════════════════════════════════════════════════════════════════════════

static int modeDetect(const std::map<std::string,std::string>& args) {
    std::string ctx_file = args.count("--ctx") ? args.at("--ctx") : "ctx.bin";
    std::string pk_file  = args.count("--pk")  ? args.at("--pk")  : "";
    std::string emk_file = args.count("--emk") ? args.at("--emk") : "emk.bin";
    std::string esk_file = args.count("--esk") ? args.at("--esk") : "esk.bin";
    std::string ct1_file = args.count("--ct1") ? args.at("--ct1") : "ct1.bin";
    std::string ct2_file = args.count("--ct2") ? args.at("--ct2") : "ct2.bin";
    std::string out_file = args.count("--out") ? args.at("--out") : "ct_result.bin";
    std::string swkfc_file = args.count("--swkfc") ? args.at("--swkfc") : "";
    std::cout << "[detect] Chargement du contexte...\n"; std::cout.flush();
    auto cc = deserializeContext(readFile(ctx_file));

    std::string btk_file = args.count("--btk") ? args.at("--btk") : "";

    std::cout << "[detect] Chargement des cles d'evaluation (emk + esk)...\n"; std::cout.flush();
    deserializeEvalMultKeys(cc, readFile(emk_file));
    deserializeEvalSumKeys(cc, readFile(esk_file));

    // loadPublicContext initialises the LWE context and loads BTK in one step.
    // EvalCompareSwitchPrecompute (called inside) may reset m_ccLWE, so BTK must be
    // loaded from within loadPublicContext while ccLWE is still captured locally.
    std::string btkData;
    if (!btk_file.empty()) {
        std::cout << "[detect] Lecture du fichier BTK...\n"; std::cout.flush();
        btkData = readFile(btk_file);
    } else {
        std::cerr << "[detect] AVERTISSEMENT: --btk absent, EvalCompareSchemeSwitching echouera\n";
    }
   
    std::cout << "[detect] Initialisation du moteur FHE (cle publique + BTK)...\n"; std::cout.flush();
    CryptoEngine engine;
    engine.loadPublicContext(cc, 8, 1, btkData);
    if (!swkfc_file.empty()) {
        std::cout << "[detect] Chargement de swkFC...\n"; std::cout.flush();
        deserializeSwkFC(cc, readFile(swkfc_file));
    } else {
        std::cerr << "[detect] AVERTISSEMENT: --swkfc absent\n";
    }

    auto swkfc_test = cc->GetSwkFC();
    if (!swkfc_test) {
        std::cerr << "[detect] ERROR: swkFC is NULL\n";
    } else {
        std::cout << "[detect] swkFC OK\n";
    }

    std::cout << "[detect] Chargement des ciphertexts Alice et Bob...\n"; std::cout.flush();
    auto ct1 = deserializeCiphertext(readFile(ct1_file));
    auto ct2 = deserializeCiphertext(readFile(ct2_file));

    std::cout << "[detect] Calcul FHE — detection de collision 3D chiffree...\n"; std::cout.flush();
    GeometryEngine geo(&engine);
    auto ct_result = geo.checkSegmentIntersection3DEncrypted(ct1, ct2);

    writeFile(out_file, serializeCiphertext(ct_result));

    size_t ss = geo.getBootstrapCount();
    std::cout << "[detect] Resultat chiffre ecrit : " << out_file << "\n";
    std::cout << "JSON_RESULT:{\"scheme_switches\":" << ss << "}\n";
    std::cout.flush();

    return 0;
}


// ══════════════════════════════════════════════════════════════════════════════
// MODE DECRYPT — Bob decrypte localement le resultat final
// ══════════════════════════════════════════════════════════════════════════════

static int modeDecrypt(const std::map<std::string,std::string>& args) {
    std::string ctx_file = args.count("--ctx") ? args.at("--ctx") : "ctx.bin";
    std::string sk_file  = args.count("--sk")  ? args.at("--sk")  : "sk.bin";
    std::string ct_file  = args.count("--ct")  ? args.at("--ct")  : "ct_result.bin";

    std::cout << "[decrypt] Chargement du contexte, de la cle secrete et du ciphertext...\n";
    std::cout.flush();

    auto cc = deserializeContext(readFile(ctx_file));
    auto sk = deserializeSecretKey(readFile(sk_file));
    auto ct = deserializeCiphertext(readFile(ct_file));

    Plaintext pt;
    cc->Decrypt(sk, ct, &pt);

    auto v = pt->GetCKKSPackedValue();
    double result = v.empty() ? 0.0 : v[0].real();
    bool collision = (result > 0.5);

    std::cout << "[decrypt] Resultat : " << (collision ? "COLLISION" : "LIBRE") << "\n";

    std::cout << "\nJSON_RESULT:{"
              << "\"value\":" << result << ","
              << "\"collision\":" << (collision ? "true" : "false")
              << "}\n";
    std::cout.flush();

    return 0;
}
static int modeServer(const std::map<std::string,std::string>& args) {
    std::string role = args.count("--role") ? args.at("--role") : "";
    if (role != "alice" && role != "bob") {
        throw std::runtime_error("Usage: --mode server --role alice|bob");
    }

    CryptoEngine engine;
    bool loaded = false;

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        try {
            auto tokens = splitWS(line);
            if (tokens.empty()) continue;

            const std::string& cmd = tokens[0];

            if (cmd == "QUIT") {
                std::cout << "OK bye\n";
                std::cout.flush();
                return 0;
            }

            if (role == "alice") {
                if (cmd == "LOAD_REMOTE") {
                    if (tokens.size() != 7) {
                        throw std::runtime_error("Usage: LOAD_REMOTE <ctx> <pk> <emk> <esk> <btk> <swkfc>");
                    }

                    auto cc = deserializeContext(readFile(tokens[1]));
                    auto pk = deserializePublicKey(readFile(tokens[2]));
                    deserializeEvalMultKeys(cc, readFile(tokens[3]));
                    deserializeEvalSumKeys(cc, readFile(tokens[4]));
                    std::string btkData = readFile(tokens[5]);


                    // Pour débloquer vite : moteur vivant avec scheme switching initialisé localement
                    engine.loadPublicContext(cc, 8, 1, btkData, pk);
                   
                    deserializeSwkFC(cc, readFile(tokens[6]));

                    loaded = true;

                    std::cout << "OK loaded_remote\n";
                    std::cout.flush();
                    continue;
                }

                if (!loaded) {
                    throw std::runtime_error("Serveur alice non initialise");
                }

                if (cmd == "ENCRYPT_PATH") {
                    if (tokens.size() != 3) {
                        throw std::runtime_error("Usage: ENCRYPT_PATH <segment.txt> <out_ct.bin>");
                    }

                    Segment seg = parseSegmentFileSimple(tokens[1]);
                    std::vector<double> coords = {
                        (double)seg.first.x,  (double)seg.first.y,  (double)seg.first.z,
                        (double)seg.second.x, (double)seg.second.y, (double)seg.second.z
                    };

                    auto ct = engine.encryptVector(coords);
                    writeFile(tokens[2], serializeCiphertext(ct));

                    std::cout << "OK encrypted\n";
                    std::cout.flush();
                    continue;
                }

                if (cmd == "DETECT_PATH") {
                    if (tokens.size() != 4 && tokens.size() != 5) {
                        throw std::runtime_error("Usage: DETECT_PATH <ct1.bin> <ct2.bin> <out_ct.bin> [same_altitude=0|1]");
                    }

                    bool sameAltitude = (tokens.size() == 5 && tokens[4] == "1");

                    auto ct1 = deserializeCiphertext(readFile(tokens[1]));
                    auto ct2 = deserializeCiphertext(readFile(tokens[2]));
                    std::cout << "[debug] isInitialized=" << engine.isInitialized() << "\n";
                    std::cout << "[debug] swkFC=" << (engine.getCKKSContext()->GetSwkFC() ? "OK" : "NULL") << "\n";
                    std::cout << "[debug] same_altitude=" << sameAltitude << "\n";
                    std::cout.flush();
                    GeometryEngine geo(&engine);
                    auto ct_result = geo.checkSegmentIntersection3DEncrypted(ct1, ct2, sameAltitude);

                    writeFile(tokens[3], serializeCiphertext(ct_result));

                    std::cout << "OK detect scheme_switches=" << geo.getBootstrapCount() << "\n";
                    std::cout.flush();
                    continue;
                }
            }

            if (role == "bob") {
                if (cmd == "LOAD_LOCAL") {
                    if (tokens.size() != 4) {
                        throw std::runtime_error("Usage: LOAD_LOCAL <ctx> <pk> <sk>");
                    }

                    auto cc = deserializeContext(readFile(tokens[1]));
                    auto pk = deserializePublicKey(readFile(tokens[2]));
                    auto sk = deserializeSecretKey(readFile(tokens[3]));

                    engine.loadContext(cc, pk, sk);
                    loaded = true;

                    std::cout << "OK loaded_local\n";
                    std::cout.flush();
                    continue;
                }

                if (!loaded) {
                    throw std::runtime_error("Serveur bob non initialise");
                }

                if (cmd == "ENCRYPT_PATH") {
                    if (tokens.size() != 3) {
                        throw std::runtime_error("Usage: ENCRYPT_PATH <segment.txt> <out_ct.bin>");
                    }

                    Segment seg = parseSegmentFileSimple(tokens[1]);
                    std::vector<double> coords = {
                        (double)seg.first.x,  (double)seg.first.y,  (double)seg.first.z,
                        (double)seg.second.x, (double)seg.second.y, (double)seg.second.z
                    };

                    auto ct = engine.encryptVector(coords);
                    writeFile(tokens[2], serializeCiphertext(ct));

                    std::cout << "OK encrypted\n";
                    std::cout.flush();
                    continue;
                }

                if (cmd == "DECRYPT_PATH") {
                    if (tokens.size() != 2) {
                        throw std::runtime_error("Usage: DECRYPT_PATH <ct.bin>");
                    }

                    auto ct = deserializeCiphertext(readFile(tokens[1]));
                    double result = engine.decryptValue(ct);
                    bool collision = (result > 0.5);

                    std::cout << "OK collision=" << (collision ? 1 : 0)
                              << " raw=" << result << "\n";
                    std::cout.flush();
                    continue;
                }
            }

            throw std::runtime_error("Commande inconnue : " + cmd);

        } catch (const std::exception& e) {
            std::cout << "ERR " << e.what() << "\n";
            std::cout.flush();
        }
    }

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
              << "\"collision\":"        << (totalCollisions > 0 ? "true" : "false") << ","
              << "\"collisions_count\":" << totalCollisions << ","
              << "\"pairs_tested\":"     << pairResults.size() << ","
              << "\"drones\":"           << paths.size() << ","
              << "\"scheme_switches\":"  << geo.getBootstrapCount() << ","
              << "\"total_time_ms\":"    << std::fixed << std::setprecision(1) << ms_total << ","
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
static int modeSelfTest(const std::map<std::string,std::string>& args) {
    std::cout << "[selftest] Initialisation moteur...\n"; std::cout.flush();

    CryptoEngine engine;
    engine.initialize();
    engine.setupSchemeSwitching();

    GeometryEngine geo(&engine);

    std::vector<double> segAlice = {
        0, 50, 30,
        6, 54, 30
    };

    std::vector<double> segBob = {
        100, 0, 30,
        98, 6, 30
    };

    std::cout << "[selftest] Chiffrement des deux segments...\n"; std::cout.flush();
    auto ctAlice = engine.encryptVector(segAlice);
    auto ctBob   = engine.encryptVector(segBob);

    std::cout << "[selftest] Lancement de checkSegmentIntersection3DEncrypted...\n"; std::cout.flush();
    auto ctResult = geo.checkSegmentIntersection3DEncrypted(ctAlice, ctBob);

    std::cout << "[selftest] Dechiffrement du resultat...\n"; std::cout.flush();
    double v = engine.decryptValue(ctResult);

    std::cout << "[selftest] Resultat brut = " << v << "\n";
    std::cout << "[selftest] Collision = " << (v > 0.5 ? "true" : "false") << "\n";
    std::cout << "[selftest] Scheme switches = " << geo.getBootstrapCount() << "\n";

    return 0;
}
// ══════════════════════════════════════════════════════════════════════════════
// MAIN
// ══════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    auto args = parseArgs(argc, argv);
    std::string mode = args.count("--mode") ? args.at("--mode") : "demo";

    try {
        if      (mode == "init")     return modeInit(args);
        else if (mode == "encrypt")  return modeEncrypt(args);
        else if (mode == "detect")   return modeDetect(args);
        else if (mode == "decrypt")  return modeDecrypt(args);
        else if (mode == "server")   return modeServer(args);
        else if (mode == "selftest") return modeSelfTest(args);
        else                         return modeDemo(args);
    } catch (const std::exception& e) {
        std::cerr << "Erreur : " << e.what() << "\n";
        return 1;
    }
}