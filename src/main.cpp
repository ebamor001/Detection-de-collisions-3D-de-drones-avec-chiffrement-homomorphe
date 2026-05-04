/**
 * main.cpp — Detection collision 3D FHE
 *
 * Modes :
 *   ./drone_fhe --mode init   --ctx ctx.bin --pk pk.bin --sk sk.bin --emk emk.bin --erk erk.bin --btk btk.bin --swkfc swkfc.bin
 *   ./drone_fhe --mode server --role alice|bob
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include "geometry.hpp"
#include "engine.hpp"
#include "engine_serial.hpp"
#include "types.hpp"
#include "io.hpp"
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
    std::string erk_file = args.count("--esk") ? args.at("--esk") : "esk.bin";
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
    writeFile(erk_file, serializeEvalSumKeys(cc));
    writeFile(btk_file, serializeBTKey(cc));
    writeFile(swkfc_file, serializeSwkFC(cc));
    std::cout << "[init] OK\n";
    std::cout << "  ctx : " << ctx_file << "\n";
    std::cout << "  pk  : " << pk_file  << "\n";
    std::cout << "  sk  : " << sk_file  << "\n";
    std::cout << "  emk : " << emk_file << "\n";
    std::cout << "  esk : " << erk_file << "\n";
    std::cout << "  btk : " << btk_file << "\n";
    std::cout << "  swkfc : " << swkfc_file << "\n";
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
                        throw std::runtime_error("Usage: LOAD_REMOTE <ctx> <pk> <emk> <erk> <btk> <swkfc>");
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
                    if (tokens.size() < 4) {
                        throw std::runtime_error("Usage: DETECT_PATH <ct1.bin> <ct2.bin> <out_ct.bin>");
                    }

                    auto ct1 = deserializeCiphertext(readFile(tokens[1]));
                    auto ct2 = deserializeCiphertext(readFile(tokens[2]));
                    std::cout << "[debug] isInitialized=" << engine.isInitialized() << "\n";
                    std::cout << "[debug] swkFC=" << (engine.getCKKSContext()->GetSwkFC() ? "OK" : "NULL") << "\n";
                    std::cout.flush();
                    GeometryEngine geo(&engine);
                    auto ct_result = geo.checkSegmentIntersection3DEncrypted(ct1, ct2);

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
// MAIN
// ══════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    auto args = parseArgs(argc, argv);
    std::string mode = args.count("--mode") ? args.at("--mode") : "demo";

    try {
        if      (mode == "init")   return modeInit(args);
        else if (mode == "server") return modeServer(args);
        else {
            std::cerr << "Mode inconnu : " << mode << "\n";
            std::cerr << "Usage: --mode init|server\n";
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Erreur : " << e.what() << "\n";
        return 1;
    }
}