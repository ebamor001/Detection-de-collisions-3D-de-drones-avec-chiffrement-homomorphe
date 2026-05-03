cmake_minimum_required(VERSION 3.16)
project(DroneCollision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenFHE REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(include)
include_directories(${OpenFHE_INCLUDE_DIRS})
include_directories(/usr/local/include/openfhe)
include_directories(/usr/local/include/openfhe/pke)
include_directories(/usr/local/include/openfhe/cc)
include_directories(/usr/local/include/openfhe/binfhe)
include_directories(/usr/local/include/openfhe/core)

# =========================
# Exécutable principal Alice/Bob
# =========================
add_executable(drone_fhe
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    src/main_alice_bob.cpp
)

target_link_libraries(drone_fhe
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

# =========================
# Ancien main centralisé
# =========================
add_executable(drone_collision
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    src/main_centralise.cpp
)

target_link_libraries(drone_collision
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

# =========================
# Tests
# =========================
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

add_executable(test_main
    tests/test_main.cpp
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
)

target_link_libraries(test_main
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)