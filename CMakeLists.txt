cmake_minimum_required(VERSION 3.0)
project(slamar)
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(/usr/include/eigen3)

#search opencv and add header file
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

set(GLUT_INCLUDE_LIB 
    "/usr/lib/x86_64-linux-gnu/libGL.so"
    "/usr/lib/x86_64-linux-gnu/libGLU.so"
     "/usr/lib/x86_64-linux-gnu/libglut.so")

set(THIRD_PARTY_LIBS ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${GLUT_INCLUDE_LIB} g2o_core g2o_stuff g2o_types_sba)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)
#add_executable(slamar main.cpp)

#install(TARGETS slamar RUNTIME DESTINATION bin)


