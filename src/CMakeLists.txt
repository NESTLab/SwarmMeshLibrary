project(test_swarmmesh)
cmake_minimum_required(VERSION 3.17.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(swarmmesh)

add_subdirectory(argos)

add_executable(test_swarmmesh swarmmesh/swarmmesh.h swarmmesh/swarmmesh.cpp testing/test_swarmmesh.cpp)
set_target_properties(test_swarmmesh PROPERTIES CXX_STANDARD 17)
target_compile_features(test_swarmmesh PUBLIC cxx_std_17)
