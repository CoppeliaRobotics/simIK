cmake_minimum_required(VERSION 3.15)
project(simExtIK)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

# XXX: without -O2, IK plugin is broken (at least on macOS):
set(CMAKE_CXX_FLAGS "-O2")

find_package(Boost REQUIRED)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.0.0 REQUIRED)
find_package(Eigen3 REQUIRED)

coppeliasim_add_plugin(
    simExtIK
    LEGACY
    SOURCES
    simExtIK.cpp
    envCont.cpp
    ../coppeliaKinematicsRoutines/ik.cpp
    ../coppeliaKinematicsRoutines/environment.cpp
    ../coppeliaKinematicsRoutines/serialization.cpp
    ../coppeliaKinematicsRoutines/ikGroupContainer.cpp
    ../coppeliaKinematicsRoutines/ikGroup.cpp
    ../coppeliaKinematicsRoutines/ikElement.cpp
    ../coppeliaKinematicsRoutines/objectContainer.cpp
    ../coppeliaKinematicsRoutines/sceneObject.cpp
    ../coppeliaKinematicsRoutines/dummy.cpp
    ../coppeliaKinematicsRoutines/joint.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/MyMath.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/7Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3X3Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4X4Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/MMatrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/scriptFunctionData.cpp
    ${COPPELIASIM_INCLUDE_DIR}/scriptFunctionDataItem.cpp
)
target_compile_definitions(simExtIK PRIVATE SIM_MATH_DOUBLE)
target_include_directories(simExtIK PRIVATE ../coppeliaKinematicsRoutines)
target_include_directories(simExtIK PRIVATE ${COPPELIASIM_INCLUDE_DIR}/simMath)
target_link_libraries(simExtIK Eigen3::Eigen)
coppeliasim_add_lua(simIK.lua)
coppeliasim_add_helpfile(simIK.htm)
coppeliasim_add_helpfile(simIK.json SUBDIR index)
