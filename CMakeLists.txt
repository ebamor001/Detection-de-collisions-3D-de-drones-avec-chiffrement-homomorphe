cmake_minimum_required(VERSION 3.16)
project(DroneCollision)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenFHE REQUIRED)

include_directories(include)
include_directories(/usr/local/include/openfhe)
include_directories(/usr/local/include/openfhe/pke)
include_directories(/usr/local/include/openfhe/cc)

include_directories(/usr/local/include/openfhe/binfhe)
include_directories(/usr/local/include/openfhe/core)
include_directories(${OpenFHE_INCLUDE_DIRS})

#link_directories(/usr/local/opt/libomp/lib)
find_package(OpenMP REQUIRED)



add_executable(drone_collision
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    src/main.cpp
)

add_executable(drone_collision_centralise
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    src/main_centralise.cpp
)
target_link_libraries(drone_collision_centralise
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)


#target_link_libraries(drone_collision ${OpenFHE_LIBRARIES} omp)
target_link_libraries(drone_collision
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)
find_package(OpenFHE REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ─── Test 1 : géométrie en clair (pas de FHE, compile et tourne vite) ─────────
add_executable(test_geometry_clear
    src/engine.cpp
    src/geometry.cpp
    src/io.cpp
    tests/test_geometry_clear.cpp
)
target_link_libraries(test_geometry_clear
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

# ─── Test 2 : oracle FHE vs clair ─────────────────────────────────────────────
add_executable(test_fhe_oracle
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    tests/test_fhe_oracle.cpp
)
target_link_libraries(test_fhe_oracle
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

# ─── Test 4 : benchmark des primitives FHE ───────────────────────────────────
add_executable(test_bench_primitives
    src/engine.cpp
    tests/test_bench_primitives.cpp
)
target_link_libraries(test_bench_primitives
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

# ─── Test 3 : benchmark de scalabilité ────────────────────────────────────────
add_executable(test_benchmark_scalability
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    tests/test_benchmark_scalability.cpp
)
target_link_libraries(test_benchmark_scalability
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)
