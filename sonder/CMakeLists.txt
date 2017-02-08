cmake_minimum_required(VERSION 2.8)

#########################
# CUDA
#
find_package(CUDA QUIET REQUIRED)

#########################
# OPENGL
#
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIR}  ${GLUT_INCLUDE_DIR} )
message(${OPENGL_INCLUDE_DIR}  ${GLUT_INCLUDE_DIR} )

#########################
# Eigen
#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

#########################
# We need this to resolve OpenGL segfault weirdness when using Eigen/OpenGLSupport
#
find_package(Threads)

#########################
# Configure this project
#
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${BASEPATH}")
include_directories("${BASEPATH}/third_party/Sophus")

#########################
# gcc flags
#
set(CMAKE_CXX_FLAGS "--std=c++11 -O2 -Wall -Wno-unused-variable -Wno-unused-function ${CMAKE_CXX_FLAGS}")

#########################
# Create sonar_viz
add_executable(sonar_viz
    src/main.cc
    src/geometry.cc
    src/gl_shapes.cc
    src/geometry.cc
    src/opengl.cc

    src/simulation/rendering.cc
    )
target_link_libraries(sonar_viz ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
