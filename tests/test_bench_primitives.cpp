/**
 * test_bench_primitives.cpp
 * =========================
 * Benchmark of individual FHE primitives (CKKS + TFHE scheme switching).
 *
 * Measured operations:
 *   1. KeyGen (CKKS + scheme switching setup)
 *   2. Encrypt scalar / vector
 *   3. Decrypt scalar / vector
 *   4. CKKS Add
 *   5. CKKS Multiply
 *   6. CKKS Rotate
 *   7. Scheme switch (compareGreaterThanZero) — dominant cost
 *   8. 3D orientation value  (2 Mult + 1 Sub)
 *   9. Full intersection test (4 scheme switches)
 *
 * Each primitive is measured N_RUNS times; mean and std-dev are reported.
 * Output: console table + CSV file results/bench_primitives.csv
 *
 * Run: ./build/test_bench_primitives
 */

#include "engine.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::duration<double, std::milli>;

// ─── Timer helper ─────────────────────────────────────────────────────────────
struct Timer {
    std::chrono::time_point<Clock> t0;
    void   start() { t0 = Clock::now(); }
    double ms()    { return Ms(Clock::now() - t0).count(); }
};

// ─── Statistics over a vector of measurements ─────────────────────────────────
struct Stats {
    double mean, stddev, min_v, max_v;
    static Stats compute(const std::vector<double>& v) {
        double m = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        double var = 0;
        for (double x : v) var += (x - m) * (x - m);
        var /= v.size();
        double mn = *std::min_element(v.begin(), v.end());
        double mx = *std::max_element(v.begin(), v.end());
        return {m, std::sqrt(var), mn, mx};
    }
};

// ─── Print a result row ────────────────────────────────────────────────────────
void print_row(const std::string& name, const Stats& s, const std::string& note = "") {
    std::cout << std::left  << std::setw(38) << name
              << std::right << std::setw(10) << std::fixed << std::setprecision(2) << s.mean
              << std::setw(10) << s.stddev
              << std::setw(10) << s.min_v
              << std::setw(10) << s.max_v
              << "   " << note << "\n";
}

void csv_row(std::ofstream& f, const std::string& name, const Stats& s, const std::string& note) {
    f << "\"" << name << "\","
      << s.mean << "," << s.stddev << "," << s.min_v << "," << s.max_v
      << ",\"" << note << "\"\n";
}

// ─── Repeat a lambda N times and return timing vector ─────────────────────────
template<typename F>
std::vector<double> repeat(F fn, int n, bool warmup = true) {
    if (warmup) fn();   // warmup run not counted
    std::vector<double> times;
    times.reserve(n);
    Timer t;
    for (int i = 0; i < n; i++) {
        t.start();
        fn();
        times.push_back(t.ms());
    }
    return times;
}

// ─── main ─────────────────────────────────────────────────────────────────────
int main()
{
    const int N_RUNS = 10;   // runs per primitive (SS is slow → keep small)
    const int N_RUNS_FAST = 30; // for cheap ops (encrypt/decrypt/arithmetic)

    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║        FHE Primitive Benchmark — CKKS + TFHE (OpenFHE)          ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    // ── 1. KeyGen + setup ────────────────────────────────────────────────────
    std::cout << "[1/3] KeyGen + Scheme Switching Setup...\n"; std::cout.flush();

    Timer t_global;
    t_global.start();

    CryptoEngine engine;
    CryptoEngine::Config cfg;
    cfg.batchSize    = 16;
    cfg.switchValues = 4;
    cfg.logQ_ccLWE   = 21;

    double t_init_start = 0;
    std::vector<double> t_keygen, t_ss_setup;

    // KeyGen (initialize without SS)
    {
        Timer t; t.start();
        engine.initialize(cfg);
        t_keygen.push_back(t.ms());
    }
    std::cout << "   KeyGen done (" << t_keygen[0] << " ms)\n"; std::cout.flush();

    // Scheme switching setup
    {
        Timer t; t.start();
        engine.setupSchemeSwitching();
        t_ss_setup.push_back(t.ms());
    }
    std::cout << "   SS Setup done (" << t_ss_setup[0] << " ms)\n\n"; std::cout.flush();

    // ── 2. Arithmetic primitives (fast, N_RUNS_FAST) ────────────────────────
    std::cout << "[2/3] Arithmetic primitives (" << N_RUNS_FAST << " runs each)...\n";
    std::cout.flush();

    double val = 3.14159;
    std::vector<double> vec(cfg.batchSize, 1.5);

    // Encrypt scalar
    auto t_enc_scalar = repeat([&]{ engine.encryptValue(val); }, N_RUNS_FAST);

    // Encrypt vector
    auto t_enc_vec    = repeat([&]{ engine.encryptVector(vec); }, N_RUNS_FAST);

    // Decrypt scalar
    auto ct_a = engine.encryptValue(val);
    auto t_dec_scalar = repeat([&]{ engine.decryptValue(ct_a); }, N_RUNS_FAST);

    // Decrypt vector
    auto ct_v = engine.encryptVector(vec);
    auto t_dec_vec = repeat([&]{ engine.decryptVector(ct_v, (uint32_t)vec.size()); }, N_RUNS_FAST);

    // CKKS Add
    auto ct_b = engine.encryptValue(2.71828);
    auto t_add = repeat([&]{ engine.add(ct_a, ct_b); }, N_RUNS_FAST);

    // CKKS Multiply
    auto t_mult = repeat([&]{ engine.mult(ct_a, ct_b); }, N_RUNS_FAST);

    // CKKS Negate
    auto t_neg = repeat([&]{ engine.negate(ct_a); }, N_RUNS_FAST);

    // CKKS Rotate
    auto t_rot = repeat([&]{ engine.rotate(ct_v, 1); }, N_RUNS_FAST);

    // 3D orientation value: dy1*dx2 - dx1*dy2 (2 Mult + 1 Sub)
    auto ct_dy1 = engine.encryptValue(100.0);
    auto ct_dx2 = engine.encryptValue(50.0);
    auto ct_dx1 = engine.encryptValue(100.0);
    auto ct_dy2 = engine.encryptValue(-50.0);
    auto t_orient = repeat([&]{
        engine.sub(engine.mult(ct_dy1, ct_dx2), engine.mult(ct_dx1, ct_dy2));
    }, N_RUNS_FAST);

    std::cout << "   Done.\n\n";

    // ── 3. Scheme switching (slow, N_RUNS) ───────────────────────────────────
    std::cout << "[3/3] Scheme switching primitives (" << N_RUNS << " runs each)...\n";
    std::cout.flush();

    auto ct_orient_val = engine.sub(engine.mult(ct_dy1, ct_dx2),
                                    engine.mult(ct_dx1, ct_dy2));

    // Single scheme switch: compareGreaterThanZero
    auto t_single_ss = repeat([&]{
        engine.compareGreaterThanZero(ct_orient_val);
    }, N_RUNS);
    std::cout << "   SS x1 done.\n"; std::cout.flush();

    // isNearZeroBand (2 scheme switches)
    auto t_near_zero = repeat([&]{
        engine.isNearZeroBand(ct_orient_val, 1e-5);
    }, N_RUNS);
    std::cout << "   isNearZeroBand done.\n"; std::cout.flush();

    // compareGtZeroPacked (1 SS batched on 4 slots)
    std::vector<double> packed_vec(4, 2500.0);
    auto ct_packed = engine.encryptVector(packed_vec);
    auto t_packed_ss = repeat([&]{
        engine.compareGtZeroPacked(ct_packed, 4);
    }, N_RUNS);
    std::cout << "   Packed SS done.\n"; std::cout.flush();

    // Full intersection test: 4 SS (orientation 4 signs)
    auto ct_o1 = engine.encryptVector(std::vector<double>(4, 2500.0));
    auto ct_o2 = engine.encryptVector(std::vector<double>(4,-2500.0));
    auto ct_o3 = engine.encryptVector(std::vector<double>(4,-2500.0));
    auto ct_o4 = engine.encryptVector(std::vector<double>(4, 2500.0));
    auto t_full_inter = repeat([&]{
        auto s1 = engine.compareGtZeroPacked(ct_o1, 4);
        auto s2 = engine.compareGtZeroPacked(ct_o2, 4);
        auto s3 = engine.compareGtZeroPacked(ct_o3, 4);
        auto s4 = engine.compareGtZeroPacked(ct_o4, 4);
        auto x12 = engine.eXor(s1, s2);
        auto x34 = engine.eXor(s3, s4);
        engine.eAnd(x12, x34);
    }, N_RUNS);
    std::cout << "   Full intersection done.\n\n";

    double total_ms = Ms(Clock::now() - t_global.t0).count();

    // ══════════════════════════════════════════════════════════════════════════
    // Print table
    // ══════════════════════════════════════════════════════════════════════════
    std::cout << std::string(88, '-') << "\n";
    std::cout << std::left  << std::setw(38) << "Operation"
              << std::right << std::setw(10) << "Mean(ms)"
              << std::setw(10) << "Std(ms)"
              << std::setw(10) << "Min(ms)"
              << std::setw(10) << "Max(ms)"
              << "   Note\n";
    std::cout << std::string(88, '-') << "\n";

    // One-time setup
    std::cout << "\n  --- Setup (one-time cost) ---\n";
    print_row("KeyGen (CKKS)",        Stats::compute(t_keygen),    "1 run");
    print_row("Scheme Switch Setup",  Stats::compute(t_ss_setup),  "1 run, switchValues=4");

    // Encryption / Decryption
    std::cout << "\n  --- Encryption / Decryption ---\n";
    print_row("Encrypt scalar",       Stats::compute(t_enc_scalar), std::to_string(N_RUNS_FAST)+" runs");
    print_row("Encrypt vector ("+std::to_string(cfg.batchSize)+" slots)",
                                      Stats::compute(t_enc_vec),    std::to_string(N_RUNS_FAST)+" runs");
    print_row("Decrypt scalar",       Stats::compute(t_dec_scalar), std::to_string(N_RUNS_FAST)+" runs");
    print_row("Decrypt vector ("+std::to_string(cfg.batchSize)+" slots)",
                                      Stats::compute(t_dec_vec),    std::to_string(N_RUNS_FAST)+" runs");

    // CKKS arithmetic
    std::cout << "\n  --- CKKS Arithmetic (no scheme switch) ---\n";
    print_row("CKKS Add",             Stats::compute(t_add),        std::to_string(N_RUNS_FAST)+" runs");
    print_row("CKKS Multiply",        Stats::compute(t_mult),       std::to_string(N_RUNS_FAST)+" runs");
    print_row("CKKS Negate",          Stats::compute(t_neg),        std::to_string(N_RUNS_FAST)+" runs");
    print_row("CKKS Rotate (1 pos)",  Stats::compute(t_rot),        std::to_string(N_RUNS_FAST)+" runs");
    print_row("3D Orientation value", Stats::compute(t_orient),     "2 Mult + 1 Sub");

    // Scheme switching
    std::cout << "\n  --- Scheme Switching CKKS → TFHE (dominant cost) ---\n";
    print_row("compareGreaterThanZero (1 SS)",
                                      Stats::compute(t_single_ss),  std::to_string(N_RUNS)+" runs");
    print_row("isNearZeroBand (2 SS)",Stats::compute(t_near_zero),  std::to_string(N_RUNS)+" runs");
    print_row("compareGtZeroPacked (1 SS, 4 slots)",
                                      Stats::compute(t_packed_ss),  std::to_string(N_RUNS)+" runs");
    print_row("Full intersection test (4 SS)",
                                      Stats::compute(t_full_inter), "4 packed SS + XOR/AND");

    std::cout << std::string(88, '-') << "\n";

    // Summary
    auto ss_stats = Stats::compute(t_single_ss);
    auto fi_stats = Stats::compute(t_full_inter);
    std::cout << "\n  Total benchmark time : " << total_ms/1000.0 << " s\n";
    std::cout << "  SS dominance         : "
              << 100.0 * fi_stats.mean / (fi_stats.mean + Stats::compute(t_orient).mean * 4)
              << "% of intersection test\n";
    std::cout << "  SS/Encrypt ratio     : x"
              << ss_stats.mean / Stats::compute(t_enc_scalar).mean << "\n\n";

    // ── CSV output ────────────────────────────────────────────────────────────
    std::string csv_path = "results/bench_primitives.csv";
    {
        // ensure results/ exists
        std::system("mkdir -p results");
        std::ofstream f(csv_path);
        f << "operation,mean_ms,stddev_ms,min_ms,max_ms,note\n";
        csv_row(f, "KeyGen",                Stats::compute(t_keygen),     "one-time");
        csv_row(f, "SS Setup",              Stats::compute(t_ss_setup),   "one-time, switchValues=4");
        csv_row(f, "Encrypt scalar",        Stats::compute(t_enc_scalar), "");
        csv_row(f, "Encrypt vector",        Stats::compute(t_enc_vec),    "batchSize=16");
        csv_row(f, "Decrypt scalar",        Stats::compute(t_dec_scalar), "");
        csv_row(f, "Decrypt vector",        Stats::compute(t_dec_vec),    "batchSize=16");
        csv_row(f, "CKKS Add",              Stats::compute(t_add),        "");
        csv_row(f, "CKKS Multiply",         Stats::compute(t_mult),       "");
        csv_row(f, "CKKS Negate",           Stats::compute(t_neg),        "");
        csv_row(f, "CKKS Rotate",           Stats::compute(t_rot),        "");
        csv_row(f, "3D Orientation value",  Stats::compute(t_orient),     "2 Mult + 1 Sub");
        csv_row(f, "compareGreaterThanZero",Stats::compute(t_single_ss),  "1 SS");
        csv_row(f, "isNearZeroBand",        Stats::compute(t_near_zero),  "2 SS");
        csv_row(f, "compareGtZeroPacked",   Stats::compute(t_packed_ss),  "1 SS batched 4 slots");
        csv_row(f, "Full intersection test",Stats::compute(t_full_inter), "4 SS + boolean");
        std::cout << "  CSV saved: " << csv_path << "\n\n";
    }

    return 0;
}
