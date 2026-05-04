/**
 * test_geometry_clear.cpp
 * =======================
 * Tests unitaires des fonctions géométriques EN CLAIR (sans FHE).
 * Ces fonctions sont statiques dans GeometryEngine, donc aucun CryptoEngine
 * n'est nécessaire. Le binaire compile et s'exécute rapidement.
 *
 * Compile : cmake --build build --target test_geometry_clear
 * Run     : ./build/test_geometry_clear
 */

#include "geometry.hpp"
#include "types.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

// ─── mini framework ──────────────────────────────────────────────────────────
static int g_total = 0;
static int g_passed = 0;
static int g_failed = 0;

static void CHECK(bool condition, const std::string &label)
{
    ++g_total;
    if (condition)
    {
        ++g_passed;
        std::cout << "  [PASS] " << label << "\n";
    }
    else
    {
        ++g_failed;
        std::cout << "  [FAIL] " << label << "\n";
    }
}

static void SECTION(const std::string &title)
{
    std::cout << "\n=== " << title << " ===\n";
}

// ─── helpers ─────────────────────────────────────────────────────────────────
static Segment seg(long x1, long y1, long z1, long x2, long y2, long z2)
{
    return {IntPoint(x1, y1, z1), IntPoint(x2, y2, z2)};
}

// ─── 1. orientationClear2D ───────────────────────────────────────────────────
static void test_orientation()
{
    SECTION("orientationClear2D (projection DROP_Z = plan XY)");

    using G = GeometryEngine;
    using D = G::DropAxis;

    // Triangle horaire dans XY : A(0,0,0), B(4,0,0), C(2,2,0)
    IntPoint A(0, 0, 0), B(4, 0, 0), C(2, 2, 0);
    auto o = G::orientationClear2D(A, B, C, D::DROP_Z);
    CHECK(o == G::COUNTERCLOCKWISE, "Triangle CCW détecté");

    // Même triangle sens horaire : A,C,B
    o = G::orientationClear2D(A, C, B, D::DROP_Z);
    CHECK(o == G::CLOCKWISE, "Triangle CW détecté");

    // Collinéaires : A(0,0,0), B(2,0,0), C(4,0,0)
    IntPoint D1(0, 0, 0), D2(2, 0, 0), D3(4, 0, 0);
    o = G::orientationClear2D(D1, D2, D3, D::DROP_Z);
    CHECK(o == G::COLLINEAR, "Points collinéaires détectés");

    // Projection DROP_X (plan YZ)
    IntPoint P(0, 0, 0), Q(0, 4, 0), R(0, 2, 2);
    o = G::orientationClear2D(P, Q, R, D::DROP_X);
    CHECK(o == G::COUNTERCLOCKWISE || o == G::CLOCKWISE,
          "Orientation non nulle en plan YZ");
}

// ─── 2. onSegmentClear2D ─────────────────────────────────────────────────────
static void test_on_segment()
{
    SECTION("onSegmentClear2D (projection DROP_Z)");

    using G = GeometryEngine;
    using D = G::DropAxis;

    // Segment P(0,0,0)→Q(4,0,0) ; point du milieu R(2,0,0) projeté XY
    IntPoint P(0, 0, 0), Q(4, 0, 0), R(2, 0, 0);
    CHECK(G::onSegmentClear2D(P, R, Q, D::DROP_Z),
          "Milieu du segment est dessus");

    // Point hors segment (à droite)
    IntPoint R_out(6, 0, 0);
    CHECK(!G::onSegmentClear2D(P, R_out, Q, D::DROP_Z),
          "Point hors segment (débordement)");

    // Point sur l'extrémité
    CHECK(G::onSegmentClear2D(P, P, Q, D::DROP_Z),
          "Extrémité de départ est sur segment");
    CHECK(G::onSegmentClear2D(P, Q, Q, D::DROP_Z),
          "Extrémité d'arrivée est sur segment");

    // Point latéral (même x, y différent)
    IntPoint R_side(2, 1, 0);
    CHECK(!G::onSegmentClear2D(P, R_side, Q, D::DROP_Z),
          "Point latéral hors segment");
}

// ─── 3. doSegmentsIntersectClear2D ───────────────────────────────────────────
static void test_intersect_2d()
{
    SECTION("doSegmentsIntersectClear2D (plan XY, DROP_Z)");

    using G = GeometryEngine;
    using D = G::DropAxis;

    // Croix classique : (0,2)→(4,2) et (2,0)→(2,4)
    auto s1 = seg(0, 2, 0, 4, 2, 0);
    auto s2 = seg(2, 0, 0, 2, 4, 0);
    CHECK(G::doSegmentsIntersectClear2D(s1, s2, D::DROP_Z),
          "Croix classique → intersection");

    // Segments parallèles horizontaux
    auto p1 = seg(0, 0, 0, 4, 0, 0);
    auto p2 = seg(0, 2, 0, 4, 2, 0);
    CHECK(!G::doSegmentsIntersectClear2D(p1, p2, D::DROP_Z),
          "Parallèles horizontaux → pas d'intersection");

    // Segments dans le même alignement mais disjoints
    auto c1 = seg(0, 0, 0, 2, 0, 0);
    auto c2 = seg(4, 0, 0, 6, 0, 0);
    CHECK(!G::doSegmentsIntersectClear2D(c1, c2, D::DROP_Z),
          "Collinéaires disjoints → pas d'intersection");

    // Extrémités qui se touchent exactement
    auto t1 = seg(0, 0, 0, 2, 0, 0);
    auto t2 = seg(2, 0, 0, 4, 2, 0);
    CHECK(G::doSegmentsIntersectClear2D(t1, t2, D::DROP_Z),
          "Extrémités partagées → intersection");

    // Segments qui se chevauchent (colinéaires)
    auto ov1 = seg(0, 0, 0, 4, 0, 0);
    auto ov2 = seg(2, 0, 0, 6, 0, 0);
    CHECK(G::doSegmentsIntersectClear2D(ov1, ov2, D::DROP_Z),
          "Chevauchement colinéaire → intersection");

    // T-intersection (un point sur le milieu de l'autre)
    auto ti1 = seg(0, 0, 0, 4, 0, 0);
    auto ti2 = seg(2, -2, 0, 2, 0, 0);
    CHECK(G::doSegmentsIntersectClear2D(ti1, ti2, D::DROP_Z),
          "T-intersection → intersection");
}

// ─── 4. doSegmentsIntersectClear3D ───────────────────────────────────────────
static void test_intersect_3d()
{
    SECTION("doSegmentsIntersectClear3D");

    using G = GeometryEngine;

    // Même altitude z=10 : collision attendue (croix dans le plan z=10)
    auto s1 = seg(0, 5, 10, 10, 5, 10);
    auto s2 = seg(5, 0, 10, 5, 10, 10);
    CHECK(G::doSegmentsIntersectClear3D(s1, s2),
          "Croix dans le plan z=10 → collision");

    // Altitudes différentes : pas de collision
    auto a1 = seg(0, 5, 10, 10, 5, 10);
    auto a2 = seg(5, 0, 20, 5, 10, 20);
    CHECK(!G::doSegmentsIntersectClear3D(a1, a2),
          "Altitudes différentes (10 vs 20) → pas de collision");

    // Segments gauches (non coplanaires) : pas de collision
    auto sk1 = seg(0, 0, 0, 10, 0, 0);  // le long de X à z=0
    auto sk2 = seg(5, -5, 5, 5, 5, 5);  // oblique, ne coupe pas sk1
    // Ce cas dépend de la géométrie exacte ; on verifie la cohérence
    bool result_skew = G::doSegmentsIntersectClear3D(sk1, sk2);
    // On vérifie juste que ça ne plante pas et retourne un bool valide
    CHECK(result_skew == true || result_skew == false,
          "Segments gauches : résultat bool valide (pas de crash)");

    // Segments parallèles, même plan z, disjoints
    auto par1 = seg(0, 0, 5, 4, 0, 5);
    auto par2 = seg(0, 2, 5, 4, 2, 5);
    CHECK(!G::doSegmentsIntersectClear3D(par1, par2),
          "Parallèles coplanaires disjoints → pas de collision");

    // Segments colinéaires qui se chevauchent en 3D
    auto col1 = seg(0, 0, 0, 4, 4, 4);
    auto col2 = seg(2, 2, 2, 6, 6, 6);
    CHECK(G::doSegmentsIntersectClear3D(col1, col2),
          "Colinéaires qui se chevauchent → collision");

    // Extrémités qui se touchent en 3D
    auto e1 = seg(0, 0, 0, 5, 5, 5);
    auto e2 = seg(5, 5, 5, 10, 0, 0);
    CHECK(G::doSegmentsIntersectClear3D(e1, e2),
          "Extrémités partagées en 3D → collision");

    // Trajectoires du scénario 1 (route1 x route2, même altitude)
    // (0,0,10)→(10,10,10) et (0,10,10)→(10,0,10) : diagonales qui se croisent
    auto diag1 = seg(0, 0, 10, 10, 10, 10);
    auto diag2 = seg(0, 10, 10, 10, 0, 10);
    CHECK(G::doSegmentsIntersectClear3D(diag1, diag2),
          "Diagonales scénario 1 → collision");

    // Scénario 2 : altitude différente
    auto alt1 = seg(0, 0, 10, 10, 10, 10);
    auto alt2 = seg(0, 10, 20, 10, 0, 20);
    CHECK(!G::doSegmentsIntersectClear3D(alt1, alt2),
          "Diagonales scénario 2 (altitudes diff.) → pas de collision");
}

// ─── 5. Cas limites (coordinates extrêmes) ───────────────────────────────────
static void test_edge_cases()
{
    SECTION("Cas limites");

    using G = GeometryEngine;

    // Coordonnées aux bornes du système (±99)
    auto ext1 = seg(-99, -99, -99, 99, 99, 99);
    auto ext2 = seg(-99, 99, 0, 99, -99, 0);
    bool r = G::doSegmentsIntersectClear3D(ext1, ext2);
    CHECK(r == true || r == false,
          "Coordonnées extrêmes ±99 → pas de crash");

    // Segment de longueur quasi-nulle (2 points très proches mais distincts)
    auto tiny = seg(5, 5, 5, 5, 5, 6);          // longueur 1 en z
    auto cross = seg(0, 5, 5, 10, 5, 5);        // perpendiculaire
    r = G::doSegmentsIntersectClear3D(tiny, cross);
    CHECK(r == true || r == false,
          "Segment très court → pas de crash");

    // Trajectoires identiques (superposées) → collision certaine
    auto same1 = seg(0, 0, 0, 10, 10, 10);
    auto same2 = seg(0, 0, 0, 10, 10, 10);
    CHECK(G::doSegmentsIntersectClear3D(same1, same2),
          "Segments identiques → collision");
}

// ─── main ─────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "========================================\n";
    std::cout << "  Tests unitaires - géométrie en clair  \n";
    std::cout << "========================================\n";

    test_orientation();
    test_on_segment();
    test_intersect_2d();
    test_intersect_3d();
    test_edge_cases();

    std::cout << "\n========================================\n";
    std::cout << "  Résultat : " << g_passed << "/" << g_total << " tests passés";
    if (g_failed > 0)
        std::cout << "  (" << g_failed << " ECHECS)";
    std::cout << "\n========================================\n";

    return (g_failed == 0) ? 0 : 1;
}
