cmake_minimum_required(VERSION 3.8)
project(test_performance)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
find_package(Eigen3 REQUIRED)

set(SRC 
utility/src/geometry_library.cpp
)

set(INCLUDE_DIR ${INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
set(INCLUDE_DIR ${INCLUDE_DIR} 
    utility/include
    test_rotation_mult/include
)

include_directories( ${INCLUDE_DIR} )

# Executables
add_executable(test_rotation_mult test_rotation_mult/src/main.cpp ${SRC})
target_link_libraries (test_rotation_mult ${Eigen_LIBRARIES})