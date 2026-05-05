/**
 * test_io.cpp
 * ===========
 * Tests unitaires du module I/O (PathIO) sans FHE.
 * Vérifie : parse_line, validate_path, generate_random_path, write/read round-trip.
 *
 * Compile : cmake --build build --target test_io
 * Run     : ./build/test_io
 */

#include "io.hpp"
#include "types.hpp"
#include <iostream>
#include <string>
#include <fstream>
#include <cstdio>

// ─── Mini framework ───────────────────────────────────────────────────────────
static int g_total = 0, g_passed = 0, g_failed = 0;

static void CHECK(bool cond, const std::string& label) {
    ++g_total;
    if (cond) { ++g_passed; std::cout << "  [PASS] " << label << "\n"; }
    else       { ++g_failed; std::cout << "  [FAIL] " << label << "\n"; }
}
static void SECTION(const std::string& t) { std::cout << "\n=== " << t << " ===\n"; }

// ─── 1. parse_line ────────────────────────────────────────────────────────────
static void test_parse_line() {
    SECTION("PathIO::parse_line");

    // Format standard
    auto p = PathIO::parse_line("(1,2,3)(4,5,6)(7,8,9)");
    CHECK(p.size() == 3,           "3 points parsés");
    CHECK(p[0].x==1 && p[0].y==2 && p[0].z==3, "Point 0 correct");
    CHECK(p[2].x==7 && p[2].y==8 && p[2].z==9, "Point 2 correct");

    // Nombres négatifs
    auto q = PathIO::parse_line("(-10,-20,-30)(0,0,0)");
    CHECK(q.size() == 2,                        "2 points avec négatifs");
    CHECK(q[0].x==-10 && q[0].y==-20 && q[0].z==-30, "Coordonnées négatives");
    CHECK(q[1].x==0   && q[1].y==0   && q[1].z==0,   "Point zéro");

    // Un seul point
    auto r = PathIO::parse_line("(5,10,15)");
    CHECK(r.size() == 1, "Un seul point");

    // Ligne vide → 0 points (warning attendu)
    auto s = PathIO::parse_line("");
    CHECK(s.size() == 0, "Ligne vide → 0 points");

    // Format invalide → 0 points
    auto t2 = PathIO::parse_line("abc def");
    CHECK(t2.size() == 0, "Format invalide → 0 points");
}

// ─── 2. validate_path ────────────────────────────────────────────────────────
static void test_validate_path() {
    SECTION("PathIO::validate_path");

    // Chemin valide
    Path ok = {IntPoint(0,0,0), IntPoint(10,10,10), IntPoint(50,50,50)};
    CHECK(PathIO::validate_path(ok), "Chemin valide (3 points)");

    // Trop peu de points
    Path one = {IntPoint(0,0,0)};
    CHECK(!PathIO::validate_path(one), "1 point → invalide");

    Path empty = {};
    CHECK(!PathIO::validate_path(empty), "Chemin vide → invalide");

    // Coordonnées hors bornes
    Path oob = {IntPoint(-100,-100,-100), IntPoint(0,0,0)};
    CHECK(!PathIO::validate_path(oob), "Coordonnées hors bornes (-100) → invalide");

    Path oob2 = {IntPoint(0,0,0), IntPoint(101,0,0)};
    CHECK(!PathIO::validate_path(oob2), "Coordonnée > 100 → invalide");

    // Points consécutifs identiques
    Path dup = {IntPoint(5,5,5), IntPoint(5,5,5), IntPoint(10,10,10)};
    CHECK(!PathIO::validate_path(dup), "Points consécutifs identiques → invalide");

    // Bornes exactes acceptées
    Path bounds = {IntPoint(-99,-99,-99), IntPoint(100,100,100)};
    CHECK(PathIO::validate_path(bounds), "Bornes [-99, 100] exactes → valide");
}

// ─── 3. generate_random_path ─────────────────────────────────────────────────
static void test_generate_random_path() {
    SECTION("PathIO::generate_random_path");

    auto p5 = PathIO::generate_random_path(5, 42);
    CHECK(p5.size() == 5, "5 points générés");
    CHECK(PathIO::validate_path(p5), "Chemin généré valide");

    // Reproductibilité (même seed → même résultat)
    auto p5b = PathIO::generate_random_path(5, 42);
    CHECK(p5[0] == p5b[0] && p5[4] == p5b[4], "Reproductible avec même seed");

    // Seeds différentes → chemins différents
    auto p5c = PathIO::generate_random_path(5, 1337);
    CHECK(!(p5[0] == p5c[0] && p5[4] == p5c[4]), "Seeds différentes → chemins différents");

    // Pas de points consécutifs identiques
    bool no_dup = true;
    for (size_t i = 1; i < p5.size(); i++)
        if (p5[i] == p5[i-1]) { no_dup = false; break; }
    CHECK(no_dup, "Aucun doublon consécutif");

    // Trop peu de points → exception
    bool threw = false;
    try { PathIO::generate_random_path(1, 0); }
    catch (...) { threw = true; }
    CHECK(threw, "Moins de 2 points → exception");
}

// ─── 4. write_path / read_single_path (round-trip) ───────────────────────────
static void test_roundtrip() {
    SECTION("write_path + read_single_path (round-trip)");

    Path original = {IntPoint(1,2,3), IntPoint(-4,-5,-6), IntPoint(7,8,9)};
    const std::string tmp = "/tmp/test_io_roundtrip.txt";

    PathIO::write_path(original, tmp);

    Path loaded;
    bool ok = true;
    try { loaded = PathIO::read_single_path(tmp); }
    catch (...) { ok = false; }
    CHECK(ok, "Lecture sans exception");
    CHECK(loaded.size() == original.size(), "Même nombre de points");
    if (loaded.size() == original.size()) {
        CHECK(loaded[0] == original[0], "Point 0 identique après round-trip");
        CHECK(loaded[1] == original[1], "Point 1 identique après round-trip");
        CHECK(loaded[2] == original[2], "Point 2 identique après round-trip");
    }

    std::remove(tmp.c_str());

    // Fichier inexistant → exception
    bool threw = false;
    try { PathIO::read_single_path("/tmp/fichier_inexistant_xyz.txt"); }
    catch (...) { threw = true; }
    CHECK(threw, "Fichier inexistant → exception");
}

// ─── main ────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "========================================\n";
    std::cout << "  Tests unitaires — Module I/O (PathIO) \n";
    std::cout << "========================================\n";

    test_parse_line();
    test_validate_path();
    test_generate_random_path();
    test_roundtrip();

    std::cout << "\n========================================\n";
    std::cout << "  Résultat : " << g_passed << "/" << g_total << " tests passés";
    if (g_failed > 0) std::cout << "  (" << g_failed << " ECHECS)";
    std::cout << "\n========================================\n";

    return (g_failed == 0) ? 0 : 1;
}
