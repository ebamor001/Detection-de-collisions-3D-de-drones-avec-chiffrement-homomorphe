cmake_minimum_required(VERSION 3.16)
project(DroneCollision)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenFHE REQUIRED)

include_directories(include)
include_directories(/usr/local/include/openfhe)
include_directories(/usr/local/include/openfhe/pke)
include_directories(/usr/local/include/openfhe/binfhe)
include_directories(/usr/local/include/openfhe/core)
include_directories(${OpenFHE_INCLUDE_DIRS})

link_directories(/usr/local/opt/libomp/lib)

add_executable(drone_collision
    src/engine.cpp
    src/geometry.cpp
    src/intersection.cpp
    src/io.cpp
    src/main.cpp
)

target_link_libraries(drone_collision ${OpenFHE_LIBRARIES} omp)
