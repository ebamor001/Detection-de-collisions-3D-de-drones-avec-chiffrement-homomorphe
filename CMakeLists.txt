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
# Exécutables
# =========================
add_executable(drone_batching
    src/main_batching.cpp
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
)
target_link_libraries(drone_batching
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)


add_executable(drone_full_fhe
    src/main_full_fhe.cpp
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
)
target_link_libraries(drone_full_fhe
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(drone_fhe
    main_alice_bob.cpp
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
)

target_link_libraries(drone_fhe
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

add_executable(test_fhe_validation
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    tests/test_fhe_validation.cpp
)

target_link_libraries(test_fhe_validation
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(test_primitives
    src/engine.cpp
    tests/test_primitives.cpp
)

target_link_libraries(test_primitives
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(test_scalability
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    tests/test_scalability.cpp
)

target_link_libraries(test_scalability
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

add_executable(test_io
    tests/test_io.cpp
    src/io.cpp
)

target_link_libraries(test_io
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(test_distance_threshold
    tests/test_distance_threshold.cpp
    src/engine.cpp
    src/geometry.cpp
    src/io.cpp
)

target_link_libraries(test_distance_threshold
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(test_integration
    tests/test_integration.cpp
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
)

target_link_libraries(test_integration
    ${OpenFHE_LIBRARIES}
    OpenMP::OpenMP_CXX
)
